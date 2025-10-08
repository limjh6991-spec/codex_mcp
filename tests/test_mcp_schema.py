from mcp_servers.isaac_controller_server import IsaacControlServer

def test_isaac_server_schema_tools():
    server = IsaacControlServer()
    schema = server.schema()
    names = [t["name"] for t in schema["tools"]]
    assert "start_sim" in names
    assert "apply_action" in names
