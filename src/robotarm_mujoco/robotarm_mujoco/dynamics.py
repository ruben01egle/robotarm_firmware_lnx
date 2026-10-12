"""
Dynamics analysis helpers: the sim's MuJoCo model, its mass matrix and the impedance gains.

Shared by push_test and docs/dynamics/mass_matrix_sampling.py.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np
from robotarm_mujoco import mjcf
import yaml

CONTROLLER = 'cartesian_impedance_controller'


def default_controllers_file():
    """Return the installed controllers.yaml, the gains the sim runs with."""
    return os.path.join(
        get_package_share_directory('robotarm_bringup'), 'config', 'controllers.yaml')


def load_model(tool='none', params_file=None, armature=None):
    """
    Build the sim's MuJoCo model.

    armature: optional {joint: kg·m²}, replaces the armature from the sim parameters.
    """
    params = mjcf.load_params(params_file)
    joints = params.setdefault('joints', {})
    for name, value in (armature or {}).items():
        joints[name] = {k: v for k, v in (joints.get(name) or {}).items()
                        if k not in ('armature', 'rotor_inertia', 'gear_ratio')}
        joints[name]['armature'] = float(value)

    robot, scene = mjcf.urdf_to_mjcf(mjcf.xacro_urdf(tool), params)
    out_dir = tempfile.mkdtemp()
    with open(os.path.join(out_dir, mjcf.ROBOT_FILE), 'w') as f:
        f.write(robot)
    scene_path = os.path.join(out_dir, mjcf.SCENE_FILE)
    with open(scene_path, 'w') as f:
        f.write(scene)
    return mujoco.MjModel.from_xml_path(scene_path)


def joint_names(m):
    return [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]


def full_mass_matrix(m, d, q):
    """Return M(q) including armature, and leave d at pose q (forward kinematics done)."""
    d.qpos[:] = q
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    return M


def load_gains(path=None):
    """Return (joint_damping, hold_stiffness) of the impedance controller."""
    with open(path or default_controllers_file()) as f:
        p = yaml.safe_load(f)[CONTROLLER]['ros__parameters']
    return np.array(p['joint_damping']), np.array(p['hold_stiffness'])
