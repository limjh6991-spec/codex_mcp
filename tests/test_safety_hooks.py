from mcp_servers.isaac_controller_server import IsaacControlServer

def test_rate_limiting_and_emergency_stop():
    server = IsaacControlServer()
    # First action within limit
    resp1 = server.apply_action([0.1, -0.05, 0.0])
    assert resp1["rate_limited"] is False
    # Action exceeding per-joint limit (0.2) -> clamp
    resp2 = server.apply_action([1.0, -0.5, 0.25])
    assert resp2["rate_limited"] is True
    assert all(abs(v) <= 0.2 + 1e-9 for v in resp2["applied"])  # clamped
    # Engage emergency stop
    server.set_emergency_stop(True)
    resp3 = server.apply_action([0.05, 0.05, 0.05])
    assert resp3["emergency_stop"] is True
    assert resp3["applied"] == [0.0, 0.0, 0.0]
    status = server.get_safety_status()
    assert status["emergency_stop"] is True
    # Release emergency stop
    server.set_emergency_stop(False)
    resp4 = server.apply_action([0.05, 0.05, 0.05])
    assert resp4["emergency_stop"] is False
