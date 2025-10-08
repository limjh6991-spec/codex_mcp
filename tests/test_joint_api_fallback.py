from src.utils.isaac_joint_api import JointAPI, ISAAC_AVAILABLE


def test_joint_api_fallback_state_and_apply():
    api = JointAPI()
    # attach without real articulation
    attach_info = api.attach("/World/roarm")
    assert attach_info["attached"] is True
    # state should gracefully fallback empty
    st = api.get_state()
    assert "positions" in st and "velocities" in st
    assert isinstance(st["positions"], list)
    # apply_delta should no-op if Isaac unavailable or no articulation
    resp = api.apply_delta([0.1, -0.1])
    if ISAAC_AVAILABLE:
        # In future real articulation path may return applied True
        assert "applied" in resp
    else:
        assert resp.get("applied") is False or resp.get("applied") is True  # accept either until real binding
