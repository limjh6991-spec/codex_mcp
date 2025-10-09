import json, socket, subprocess, sys, time, os, random
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / 'scripts' / 'ipc_policy_gateway.py'
JOINT_SPEC = Path(__file__).resolve().parent.parent / 'assets' / 'roarm_m3' / 'joint_spec.json'

def _pick_port():
    import socket as _s, random as _r
    for _ in range(30):
        p = _r.randint(46000, 48000)
        with _s.socket(_s.AF_INET, _s.SOCK_STREAM) as s:
            try:
                s.bind(('127.0.0.1', p))
                return p
            except OSError:
                continue
    raise RuntimeError('no_free_port')

def _wait(port):
    start = time.time()
    while time.time() - start < 5:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.3) as s:
                s.sendall(b'{"type":"ping"}\n')
                if s.recv(4096):
                    return
        except Exception:
            time.sleep(0.05)
    raise RuntimeError('gateway_not_ready')

def test_joint_spec_mismatch_flag(tmp_path):
    port = _pick_port()
    proc = subprocess.Popen([sys.executable, str(SCRIPT), '--port', str(port), '--joint-spec', str(JOINT_SPEC)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        _wait(port)
        # Create and load a dummy linear policy spec so gateway enters action path
        policy_path = tmp_path / 'dummy_policy.json'
        policy_path.write_text(json.dumps({'type':'linear_dummy','action_dim':6}))
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            load_msg = json.dumps({"type":"load_policy","path": str(policy_path)}).encode()+b"\n"
            s.sendall(load_msg)
            _ = s.recv(4096)  # consume ack
        # Send q shorter than spec (spec has 6 joints)
        msg = json.dumps({"type":"obs","data":{"q":[0,1,2],"dq":[0,0,0]}}).encode()+b"\n"
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            s.sendall(msg)
            line = s.recv(16384)
        resp = json.loads(line.decode())
        data = resp.get('data', {})
        assert data.get('joint_spec_mismatch') is True
        assert data.get('expected_dof') == 6
        assert data.get('observed_dof') == 3
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
