"""
URDF to MJCF conversion of the robotarm.

MuJoCo's own URDF compiler does the conversion (kinematics, inertials, limits, meshes), this
module only prepares the URDF for it and adds what a URDF cannot express:

- visual and collision geoms in separate classes (visual: group 2, no contacts; collision: group 3)
- URDF material colours on the visual geoms (the URDF compiler drops them)
- sites for the links MuJoCo fuses into their parent (fixed joints: Base_1, flange, tcp, tool)
- per joint armature, damping and frictionloss, and solver options, from config/sim_params.yaml
- one <motor> actuator per joint, named like the joint, limited to the URDF effort limit
- a scene.xml with floor and light that includes the robot

The output references the meshes in robotarm_description's share directory by absolute path, so it
is meant to be regenerated (sim.launch.py does that on every start), not committed.
"""

import argparse
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np
import xacro
import yaml

ROBOT_FILE = 'robotarm.xml'
SCENE_FILE = 'scene.xml'

# URDF elements MuJoCo's URDF compiler does not need
_NON_URDF_TAGS = ('ros2_control', 'transmission', 'gazebo')


def share_dir():
    return get_package_share_directory('robotarm_mujoco')


def default_params_file():
    return os.path.join(share_dir(), 'config', 'sim_params.yaml')


def xacro_urdf(tool='none', mujoco_model='', headless='false'):
    """Return the URDF of the simulated robot, also the robot_description of the sim."""
    path = os.path.join(share_dir(), 'urdf', 'robotarm_sim.urdf.xacro')
    doc = xacro.process_file(path, mappings={
        'tool': tool,
        'mujoco_model': mujoco_model,
        'headless': headless,
    })
    return doc.toxml()


def load_params(path=None):
    with open(path or default_params_file()) as f:
        return yaml.safe_load(f) or {}


def joint_armature(joint_params):
    """Return the armature of a sim_params.yaml joint: as given, or rotor_inertia * gear_ratio²."""
    if 'armature' in joint_params:
        if 'rotor_inertia' in joint_params or 'gear_ratio' in joint_params:
            raise ValueError('give either armature or rotor_inertia + gear_ratio, not both')
        return float(joint_params['armature'])
    if 'rotor_inertia' in joint_params or 'gear_ratio' in joint_params:
        if 'rotor_inertia' not in joint_params or 'gear_ratio' not in joint_params:
            raise ValueError('rotor_inertia and gear_ratio must be given together')
        return float(joint_params['rotor_inertia']) * float(joint_params['gear_ratio']) ** 2
    return 0.0


def _resolve_uri(uri):
    if uri.startswith('package://'):
        package, _, rest = uri[len('package://'):].partition('/')
        return os.path.join(get_package_share_directory(package), rest)
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return uri


def _origin_transform(origin):
    """Return the 4x4 transform of a URDF <origin> (None: identity), R = Rz(y) Ry(p) Rx(r)."""
    T = np.eye(4)
    if origin is None:
        return T
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    r, p, y = (float(v) for v in origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    T[:3, 3] = xyz
    return T


def _fmt(values):
    return ' '.join(f'{v:.9g}' for v in values)


def _visual_colours(urdf_root):
    """Map mesh name (file stem, like MuJoCo's mesh assets) to the rgba of its URDF visual."""
    materials = {}
    for m in urdf_root.findall('material'):
        c = m.find('color')
        if c is not None:
            materials[m.get('name')] = c.get('rgba')
    colours = {}
    for visual in urdf_root.iter('visual'):
        mesh = visual.find('geometry/mesh')
        material = visual.find('material')
        if mesh is None or material is None:
            continue
        c = material.find('color')
        rgba = c.get('rgba') if c is not None else materials.get(material.get('name'))
        if rgba:
            colours[os.path.splitext(os.path.basename(mesh.get('filename')))[0]] = rgba
    return colours


def _prepare_urdf(urdf):
    """Strip ros2_control & co., resolve mesh paths and add the MuJoCo compiler settings."""
    root = ET.fromstring(urdf)
    for tag in _NON_URDF_TAGS:
        for element in root.findall(tag):
            root.remove(element)
    for mesh in root.iter('mesh'):
        mesh.set('filename', _resolve_uri(mesh.get('filename')))
    mujoco_element = ET.SubElement(root, 'mujoco')
    # keep visuals (they become non colliding geoms), keep absolute mesh paths,
    # use the CAD inertials as they are
    ET.SubElement(mujoco_element, 'compiler', {
        'discardvisual': 'false',
        'strippath': 'false',
        'balanceinertia': 'false',
    })
    return root


def _compile_urdf(urdf_root):
    """Compile with MuJoCo's URDF compiler, return the MJCF it saves."""
    try:
        model = mujoco.MjModel.from_xml_string(ET.tostring(urdf_root, encoding='unicode'))
    except ValueError as e:
        raise ValueError(f'MuJoCo rejected the URDF: {e}') from e
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, ROBOT_FILE)
        mujoco.mj_saveLastXML(path, model)
        return ET.parse(path).getroot()


def _add_geom_classes(mjcf, colours):
    default = mjcf.find('default')
    if default is None:
        default = ET.Element('default')
        mjcf.insert(1, default)
    visual = ET.SubElement(default, 'default', {'class': 'visual'})
    ET.SubElement(visual, 'geom', {
        'group': '2', 'contype': '0', 'conaffinity': '0', 'density': '0'})
    collision = ET.SubElement(default, 'default', {'class': 'collision'})
    ET.SubElement(collision, 'geom', {'group': '3', 'rgba': '0.8 0.4 0.1 0.4'})

    for geom in mjcf.find('worldbody').iter('geom'):
        # the URDF compiler marks visuals with contype="0" conaffinity="0" group="1"
        is_visual = geom.get('contype') == '0' and geom.get('conaffinity') == '0'
        for attribute in ('contype', 'conaffinity', 'group', 'density'):
            geom.attrib.pop(attribute, None)
        geom.set('class', 'visual' if is_visual else 'collision')
        if is_visual and geom.get('mesh') in colours:
            geom.set('rgba', colours[geom.get('mesh')])


def _add_fused_link_sites(mjcf, urdf_root):
    """Add a site per URDF link MuJoCo fused away, in the body it was fused into."""
    bodies = {b.get('name'): b for b in mjcf.iter('body')}
    bodies['world'] = mjcf.find('worldbody')
    parent_joint = {j.find('child').get('link'): j for j in urdf_root.findall('joint')}

    for link in urdf_root.findall('link'):
        name = link.get('name')
        if name in bodies:
            continue
        T = np.eye(4)
        current = name
        while current not in bodies:
            joint = parent_joint.get(current)
            if joint is None:   # the root link, fused into the world
                current = 'world'
                break
            if joint.get('type') != 'fixed':
                raise ValueError(f'link {current} has a moving joint but no MuJoCo body')
            T = _origin_transform(joint.find('origin')) @ T
            current = joint.find('parent').get('link')
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, T[:3, :3].flatten())
        ET.SubElement(bodies[current], 'site', {
            'name': name, 'pos': _fmt(T[:3, 3]), 'quat': _fmt(quat)})


def _apply_joint_params(mjcf, joint_params):
    joints = {j.get('name'): j for j in mjcf.iter('joint')}
    unknown = sorted(set(joint_params) - set(joints))
    if unknown:
        raise ValueError(f'sim_params.yaml names joints that are not in the URDF: {unknown}')
    for name, params in joint_params.items():
        params = params or {}
        try:
            armature = joint_armature(params)
        except ValueError as e:
            raise ValueError(f'sim_params.yaml, joint {name}: {e}') from e
        joint = joints[name]
        joint.set('armature', f'{armature:.9g}')
        for key in ('damping', 'frictionloss'):
            if key in params:
                joint.set(key, f'{float(params[key]):.9g}')


def _add_actuators(mjcf):
    actuator = ET.SubElement(mjcf, 'actuator')
    for joint in mjcf.iter('joint'):
        name = joint.get('name')
        force_range = joint.get('actuatorfrcrange')
        if force_range is None:
            raise ValueError(f'joint {name} has no effort limit in the URDF')
        ET.SubElement(actuator, 'motor', {'name': name, 'joint': name, 'ctrlrange': force_range})


def _add_options(mjcf, simulation, contact_exclude):
    option = ET.Element('option', {
        'timestep': f"{float(simulation.get('timestep', 0.001)):.9g}",
        'integrator': str(simulation.get('integrator', 'implicitfast')),
    })
    compiler = mjcf.find('compiler')
    mjcf.insert(list(mjcf).index(compiler) + 1 if compiler is not None else 0, option)

    # The root link is fused into the world, and MuJoCo's parent/child contact filter does not
    # apply to the world. Without these excludes the base meshes permanently touch the first link.
    pairs = [('world', body.get('name')) for body in mjcf.find('worldbody').findall('body')]
    pairs += [tuple(pair) for pair in contact_exclude]
    contact = ET.SubElement(mjcf, 'contact')
    for body1, body2 in pairs:
        ET.SubElement(contact, 'exclude', {'body1': body1, 'body2': body2})


def scene_xml(mouse_stiffness=10.0):
    return f"""<mujoco model="robotarm scene">
  <include file="{ROBOT_FILE}"/>

  <statistic center="0 0 0.4" extent="1.2"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="135" elevation="-25"/>
    <map stiffness="{float(mouse_stiffness):.9g}"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
        width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
        rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"
        reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""


def urdf_to_mjcf(urdf, params):
    """Convert a URDF string to (robot MJCF, scene MJCF), the scene includes ROBOT_FILE."""
    urdf_root = _prepare_urdf(urdf)
    mjcf = _compile_urdf(urdf_root)

    _add_geom_classes(mjcf, _visual_colours(urdf_root))
    _add_fused_link_sites(mjcf, urdf_root)
    _apply_joint_params(mjcf, params.get('joints') or {})
    _add_actuators(mjcf)
    _add_options(
        mjcf, params.get('simulation') or {}, params.get('contact_exclude') or [])

    ET.indent(mjcf, space='  ')
    simulation = params.get('simulation') or {}
    return ET.tostring(mjcf, encoding='unicode') + '\n', scene_xml(
        simulation.get('mouse_stiffness', 10.0))


def generate(out_dir, tool='none', params_file=None):
    """Convert the sim URDF for the given tool into out_dir, return the scene file path."""
    robot, scene = urdf_to_mjcf(xacro_urdf(tool), load_params(params_file))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, ROBOT_FILE), 'w') as f:
        f.write(robot)
    scene_path = os.path.join(out_dir, SCENE_FILE)
    with open(scene_path, 'w') as f:
        f.write(scene)
    return scene_path


def main():
    parser = argparse.ArgumentParser(
        description='Convert the robotarm URDF to a MuJoCo model (MJCF)')
    parser.add_argument('-o', '--output', default='mjcf',
                        help='output directory (default: ./mjcf)')
    parser.add_argument('--tool', default='none',
                        help='file name in robotarm_description/urdf/tools/ without .xacro')
    parser.add_argument('--params', default=None,
                        help='sim parameter yaml (default: the installed sim_params.yaml)')
    args = parser.parse_args()

    scene = generate(args.output, args.tool, args.params)
    print(scene)


if __name__ == '__main__':
    main()
