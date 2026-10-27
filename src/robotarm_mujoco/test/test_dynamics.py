"""Tests of the dynamics helpers shared by push_test and the mass matrix sampling."""

import os

import mujoco
import numpy as np
import pytest
from robotarm_mujoco import dynamics, mjcf
import yaml


@pytest.fixture(scope='module')
def model():
    return dynamics.load_model()


def test_mass_matrix_is_symmetric_positive_definite(model):
    d = mujoco.MjData(model)
    rng = np.random.default_rng(0)
    for _ in range(10):
        q = rng.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1])
        M = dynamics.full_mass_matrix(model, d, q)
        np.testing.assert_allclose(M, M.T, atol=1e-12)
        assert np.all(np.linalg.eigvalsh(M) > 0)


def test_armature_override_adds_to_the_diagonal(model):
    joint = dynamics.joint_names(model)[-1]
    q = np.zeros(model.nq)
    light = dynamics.load_model(armature={joint: 0.0})
    heavy = dynamics.load_model(armature={joint: 0.5})
    m_light = dynamics.full_mass_matrix(light, mujoco.MjData(light), q)
    m_heavy = dynamics.full_mass_matrix(heavy, mujoco.MjData(heavy), q)
    assert m_heavy[-1, -1] - m_light[-1, -1] == pytest.approx(0.5)


def test_armature_override_of_unknown_joint_is_an_error():
    with pytest.raises(ValueError, match='axis9'):
        dynamics.load_model(armature={'axis9': 0.1})


def test_gains_have_one_entry_per_joint(model):
    damping, stiffness = dynamics.load_gains()
    assert damping.shape == (model.nv,) and stiffness.shape == (model.nv,)
    assert np.all(damping >= 0) and np.all(stiffness >= 0)


def test_wrench_plugin_is_configured():
    with open(os.path.join(mjcf.share_dir(), 'config', 'sim_plugins.yaml')) as f:
        params = yaml.safe_load(f)['mujoco_ros2_control_node']['ros__parameters']
    types = [plugin['type'] for plugin in params['mujoco_plugins'].values()]
    assert 'mujoco_ros2_control_plugins/ExternalWrenchPlugin' in types


def test_servo_plugin_is_configured():
    # the hardware plugin refuses to start without it
    with open(os.path.join(mjcf.share_dir(), 'config', 'sim_plugins.yaml')) as f:
        params = yaml.safe_load(f)['mujoco_ros2_control_node']['ros__parameters']
    types = [plugin['type'] for plugin in params['mujoco_plugins'].values()]
    assert 'robotarm_mujoco_hardware/ServoPlugin' in types
