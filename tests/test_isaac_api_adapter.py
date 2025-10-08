from src.adapters import isaac_api_adapter as ada


def test_adapter_flags_and_functions_without_isaac():
    # In CI Isaac likely not available; we only enforce graceful behavior
    world = ada.create_world()
    # world may be None when Isaac absent
    art = ada.find_articulation(world, "/World/roarm")
    state = ada.get_joint_state(art)
    resp = ada.apply_joint_delta(None, [0.1, -0.2, 0.3], lower=[-0.05]*3, upper=[0.05]*3)
    assert isinstance(state, dict)
    assert "positions" in state and "velocities" in state
    assert resp["applied"] is False
    assert resp["note"] == "no_articulation"
