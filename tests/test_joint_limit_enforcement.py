import json, os, socket, threading, time
from scripts import ipc_policy_gateway as gw

# We will test the helper directly (faster, no socket) and joint spec loader.

def test_apply_joint_limits_basic():
    # Inject fake joint limits
    gw._joint_limits = [(-1.0, 1.0), (-2.0, 2.0), (0.0, 3.0)]
    q = [0.5, -1.5, 2.5]
    # Proposed deltas push some joints out of bounds
    delta = [0.7, -1.0, 1.0]  # q+delta => [1.2, -2.5, 3.5]
    new_delta, clipped, indices = gw._apply_joint_limits(q, delta[:])
    assert clipped is True
    # Joint0 clipped to 1.0 => delta becomes 0.5 (1.0 - 0.5)
    # Joint1 clipped to -2.0 => delta becomes -0.5 (-2.0 - -1.5)
    # Joint2 clipped to 3.0 => delta becomes 0.5 (3.0 - 2.5)
    assert new_delta == [0.5, -0.5, 0.5]
    assert set(indices) == {0,1,2}

def test_load_joint_spec(tmp_path):
    spec_path = tmp_path / 'joint_spec.json'
    spec_path.write_text(json.dumps({
        'joints':[{'lower':-1,'upper':1},{'lower':0,'upper':2}]
    }))
    ok, detail = gw._load_joint_spec(str(spec_path))
    assert ok, detail
    assert gw._joint_limits == [(-1.0,1.0),(0.0,2.0)]
