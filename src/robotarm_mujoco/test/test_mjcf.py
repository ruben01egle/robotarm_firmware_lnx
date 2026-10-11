"""
Checks of the URDF -> MJCF conversion against Pinocchio, built from the same URDF.

robotarm_rbd is checked against Pinocchio in its own tests, so agreement here means the simulated
dynamics are the ones the controllers compute with.
"""

import glob
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
import pinocchio as pin
import pytest
from robotarm_mujoco import mjcf
import xacro

N_SAMPLES = 20
# mj_saveLastXML writes 6 significant digits, so the MJCF matches the URDF to about 1e-6 relative
RTOL = 1e-5
ATOL = 1e-5


def tools():
    tools_dir = os.path.join(get_package_share_directory('robotarm_description'), 'urdf', 'tools')
    paths = glob.glob(os.path.join(tools_dir, '*.xacro'))
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in paths)


class Sim:
    """Converted model and the Pinocchio reference for one tool."""

    def __init__(self, tool):
        self.urdf = mjcf.xacro_urdf(tool)
        self.params = mjcf.load_params()
        self.tmp = tempfile.TemporaryDirectory()
        robot, scene = mjcf.urdf_to_mjcf(self.urdf, self.params)
        for name, content in ((mjcf.ROBOT_FILE, robot), (mjcf.SCENE_FILE, scene)):
            with open(os.path.join(self.tmp.name, name), 'w') as f:
                f.write(content)
        self.scene_path = os.path.join(self.tmp.name, mjcf.SCENE_FILE)
        self.m = mujoco.MjModel.from_xml_path(self.scene_path)
        self.d = mujoco.MjData(self.m)
        self.pm = pin.buildModelFromXML(self.urdf)
        self.pd = self.pm.createData()

    def joint_names(self):
        joint = mujoco.mjtObj.mjOBJ_JOINT
        return [mujoco.mj_id2name(self.m, joint, i) for i in range(self.m.njnt)]

    def random_q(self, rng):
        return rng.uniform(self.m.jnt_range[:, 0], self.m.jnt_range[:, 1])

    def forward(self, q):
        self.d.qpos[:] = q
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)


@pytest.fixture(scope='module', params=tools())
def sim(request):
    return Sim(request.param)


def test_joints_match_urdf(sim):
    pin_names = list(sim.pm.names)[1:]  # 0 is the universe
    assert sim.joint_names() == pin_names
    np.testing.assert_allclose(sim.m.jnt_range[:, 0], sim.pm.lowerPositionLimit, atol=1e-5)
    np.testing.assert_allclose(sim.m.jnt_range[:, 1], sim.pm.upperPositionLimit, atol=1e-5)


def test_one_motor_per_joint_at_the_effort_limit(sim):
    names = [mujoco.mj_id2name(sim.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(sim.m.nu)]
    assert names == sim.joint_names()
    assert list(sim.m.actuator_trnid[:, 0]) == list(range(sim.m.njnt))
    np.testing.assert_allclose(sim.m.actuator_ctrlrange[:, 1], sim.pm.effortLimit)
    np.testing.assert_allclose(sim.m.actuator_ctrlrange[:, 0], -sim.pm.effortLimit)
    np.testing.assert_allclose(sim.m.actuator_gear[:, 0], 1.0)


def test_armature_from_params(sim):
    joints = sim.params['joints']
    expected = [mjcf.joint_armature(joints.get(n) or {}) for n in sim.joint_names()]
    np.testing.assert_allclose(sim.m.dof_armature, expected)
    axis2 = joints['axis2']
    expected_axis2 = axis2['rotor_inertia'] * axis2['gear_ratio'] ** 2
    assert sim.m.dof_armature[1] == pytest.approx(expected_axis2)


def test_gravity_torques_match_pinocchio(sim):
    rng = np.random.default_rng(1)
    for _ in range(N_SAMPLES):
        q = sim.random_q(rng)
        sim.forward(q)
        expected = pin.computeGeneralizedGravity(sim.pm, sim.pd, q)
        np.testing.assert_allclose(
            sim.d.qfrc_bias, expected, rtol=RTOL, atol=ATOL, err_msg=f'q = {q}')


def test_mass_matrix_matches_pinocchio(sim):
    # checks masses, centres of mass, inertia tensors and joint frames all at once
    rng = np.random.default_rng(2)
    M = np.zeros((sim.m.nv, sim.m.nv))
    for _ in range(N_SAMPLES):
        q = sim.random_q(rng)
        sim.forward(q)
        mujoco.mj_fullM(sim.m, sim.d, M)
        expected = pin.crba(sim.pm, sim.pd, q)
        expected = np.triu(expected) + np.triu(expected, 1).T
        np.testing.assert_allclose(
            M - np.diag(sim.m.dof_armature), expected, rtol=RTOL, atol=ATOL, err_msg=f'q = {q}')


def test_tcp_site_matches_pinocchio(sim):
    tcp = mujoco.mj_name2id(sim.m, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
    assert tcp >= 0
    frame = sim.pm.getFrameId('tcp')
    rng = np.random.default_rng(3)
    for _ in range(N_SAMPLES):
        q = sim.random_q(rng)
        sim.forward(q)
        pin.framesForwardKinematics(sim.pm, sim.pd, q)
        expected = sim.pd.oMf[frame]
        np.testing.assert_allclose(sim.d.site_xpos[tcp], expected.translation, atol=ATOL)
        np.testing.assert_allclose(
            sim.d.site_xmat[tcp].reshape(3, 3), expected.rotation, atol=ATOL)


def test_gravity_compensation_holds_pose(sim):
    # tau = g(q) from the reference model, no feedback: the arm must stay where it is. A sign,
    # joint order or unit mismatch between the controllers' model and the sim makes it fall.
    rng = np.random.default_rng(4)
    held = 0
    while held < 5:
        q0 = sim.random_q(rng)
        sim.forward(q0)
        if sim.d.ncon > 0:
            continue
        g = pin.computeGeneralizedGravity(sim.pm, sim.pd, q0)
        if np.any(np.abs(g) > sim.pm.effortLimit):
            continue
        sim.d.ctrl[:] = 0.0
        for _ in range(2000):  # 2 s
            sim.d.ctrl[:] = pin.computeGeneralizedGravity(sim.pm, sim.pd, sim.d.qpos)
            mujoco.mj_step(sim.m, sim.d)
        np.testing.assert_allclose(sim.d.qpos, q0, atol=1e-3, err_msg=f'q0 = {q0}')
        held += 1


def test_passive_drop_stays_finite(sim):
    sim.forward(np.zeros(sim.m.nq))
    sim.d.ctrl[:] = 0.0
    for _ in range(3000):
        mujoco.mj_step(sim.m, sim.d)
    assert np.all(np.isfinite(sim.d.qpos))
    assert np.all(np.isfinite(sim.d.qvel))


def test_no_contact_between_base_and_first_link(sim):
    sim.forward(np.zeros(sim.m.nq))
    assert sim.d.ncon == 0


def test_vendored_mujoco_compiles_the_scene(sim):
    # the hardware interface uses mujoco_vendor's library, which may be older than the pip package
    compile_tool = os.path.join(
        get_package_prefix('mujoco_vendor'), 'lib', 'mujoco_vendor', 'compile')
    out = os.path.join(sim.tmp.name, 'compiled.mjb')
    result = subprocess.run([compile_tool, sim.scene_path, out], capture_output=True, text=True)
    assert result.returncode == 0 and os.path.exists(out), result.stdout + result.stderr


def ros2_control_joints(urdf):
    return [j.get('name') for j in ET.fromstring(urdf).find('ros2_control').findall('joint')]


def test_sim_ros2_control_joints_match_the_hardware():
    # the sim's ros2_control block duplicates the joint list of robotarm_description's
    path = os.path.join(
        get_package_share_directory('robotarm_description'), 'urdf', 'robotarm.urdf.xacro')
    hardware = ros2_control_joints(xacro.process_file(path).toxml())
    simulated = ros2_control_joints(mjcf.xacro_urdf())
    assert simulated == hardware
    sim_root = ET.fromstring(mjcf.xacro_urdf())
    for joint in sim_root.find('ros2_control').findall('joint'):
        assert [c.get('name') for c in joint.findall('command_interface')] == ['effort']


def test_robot_description_carries_the_model_path():
    urdf = mjcf.xacro_urdf(mujoco_model='/some/scene.xml', headless='true')
    hardware = ET.fromstring(urdf).find('ros2_control/hardware')
    params = {p.get('name'): p.text for p in hardware.findall('param')}
    assert params == {'mujoco_model': '/some/scene.xml', 'headless': 'true'}


def test_unknown_joint_in_params_is_an_error():
    with pytest.raises(ValueError, match='axis9'):
        mjcf.urdf_to_mjcf(mjcf.xacro_urdf(), {'joints': {'axis9': {'damping': 1.0}}})


@pytest.mark.parametrize('joint_params', [
    {'armature': 0.1, 'rotor_inertia': 1e-4, 'gear_ratio': 10.0},
    {'rotor_inertia': 1e-4},
    {'gear_ratio': 10.0},
])
def test_inconsistent_armature_params_are_an_error(joint_params):
    with pytest.raises(ValueError):
        mjcf.joint_armature(joint_params)


def test_armature_from_rotor_inertia_and_ratio():
    assert mjcf.joint_armature({'rotor_inertia': 2e-4, 'gear_ratio': 10.0}) == pytest.approx(0.02)
    assert mjcf.joint_armature({'armature': 0.3}) == 0.3
    assert mjcf.joint_armature({}) == 0.0
