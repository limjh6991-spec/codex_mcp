import json, socket, subprocess, sys, time, pathlib

GW = pathlib.Path(__file__).resolve().parent.parent / 'scripts' / 'ipc_policy_gateway.py'

def pick_port():
    import socket as s, random as r
    for _ in range(30):
        p = r.randint(48000, 50000)
        with s.socket(s.AF_INET, s.SOCK_STREAM) as sock:
            try:
                sock.bind(('127.0.0.1', p))
                return p
            except OSError:
                continue
    raise RuntimeError('no_port')

def wait_ready(port):
    start = time.time()
    while time.time() - start < 5:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.3) as s:
                s.sendall(b'{"type":"ping"}\n')
                if s.recv(4096):
                    return
        except Exception:
            time.sleep(0.05)
    raise RuntimeError('not_ready')

# We simulate deadline misses by using a tiny deadline and sending many obs; model inference is fast but total handling should still be > threshold sometimes.
# If not naturally miss, we could adapt by setting deadline extremely small (e.g., 0.01ms) which is guaranteed to miss post-processing.

def test_deadline_escalation():
    port = pick_port()
    import tempfile, os
    log_dir = tempfile.mkdtemp(prefix='gw_logs_')
    # Use 1.0 ms deadline (instead of 0.001 ms) for stable measurable overshoot on typical Python scheduling
    proc = subprocess.Popen([sys.executable, str(GW), '--port', str(port), '--deadline-ms', '1.0', '--deadline-escalate-threshold', '3', '--degrade-mode-ratio', '0.5', '--log-dir', log_dir], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        wait_ready(port)
        # Load dummy policy
        policy_spec = {'type':'linear_dummy','action_dim':4}
        import tempfile, json as _j
        tmp = tempfile.NamedTemporaryFile('w', delete=False, suffix='.json')
        tmp.write(_j.dumps(policy_spec))
        tmp.flush()
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            s.sendall(json.dumps({'type':'load_policy','path':tmp.name}).encode()+b'\n')
            s.recv(4096)
        # Send several obs to trigger consecutive misses
        escalate_seen = False
        for i in range(8):  # send a few more to ensure escalation threshold crossed
            with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
                s.sendall(json.dumps({'type':'obs','data':{'q':[0,1,2,3],'dq':[0,0,0,0]}}).encode()+b'\n')
                line = s.recv(16384)
            resp = json.loads(line.decode())
            data = resp.get('data', {})
            if data.get('deadline_miss'):
                # escalation advisory not directly in action payload; rely on counters after threshold
                pass
        # After loop, metrics file should reflect escalation counter increased >=1
        # Force flush via admin command then get counters deterministically
        def admin(op):
            with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
                s.sendall(json.dumps({'type':'admin','op':op}).encode()+b'\n')
                return json.loads(s.recv(16384).decode())
        admin('flush_metrics')
        counters_resp = admin('get_counters')
        cnts = counters_resp.get('data', {})
        miss_cnt = cnts.get('deadline_miss')
        esc_cnt = cnts.get('deadline_escalation_events')
        assert miss_cnt and miss_cnt >= 3, f'deadline_miss did not accumulate (deadline_miss={miss_cnt})'
        assert esc_cnt is not None and esc_cnt >= 1, f'escalation not recorded (escalation={esc_cnt}, counters={cnts})'
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
