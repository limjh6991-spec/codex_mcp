from mcp_servers.isaac_controller_server import IsaacControlServer

def test_mcp_joint_tools_placeholder():
    server = IsaacControlServer()
    server.start_sim()
    st = server.get_joint_state()
    assert "positions" in st
    resp = server.set_joint_targets([0.1, -0.1, 0.2])
    assert resp["status"] == "ok"
    delta = server.apply_action([0.05, 0.05, -0.1])
    assert "applied" in delta
