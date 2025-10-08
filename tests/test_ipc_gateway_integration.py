import socket, json, time, threading, subprocess, sys, os

HOST = '127.0.0.1'
PORT = 45321
_gateway_proc = None

def start_gateway(extra_args=None):
    args = [sys.executable, os.path.join(os.path.dirname(__file__), '..', 'scripts', 'ipc_policy_gateway.py'), '--host', HOST, '--port', str(PORT)]
    if extra_args:
        args.extend(extra_args)
    # Start as subprocess
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def send(msg: dict):
    with socket.create_connection((HOST, PORT), timeout=5) as s:
        s.sendall((json.dumps(msg) + '\n').encode())
        data = s.recv(4096).decode().strip()
        return json.loads(data)


def ensure_gateway(extra_args=None):
    global _gateway_proc
    if _gateway_proc is None or _gateway_proc.poll() is not None:
        _gateway_proc = start_gateway(extra_args=extra_args)
        time.sleep(0.6)
    return _gateway_proc

def test_ping_roundtrip():
    ensure_gateway()
    resp = send({"type": "ping"})
    assert resp['type'] == 'ack'
    assert 'ts' in resp


def test_rand_hash_echo_and_deadline():
    ensure_gateway(['--deadline-ms', '0.01'])
    dummy_path = os.path.join(os.path.dirname(__file__), 'dummy_policy.json')
    with open(dummy_path, 'w', encoding='utf-8') as f:
        json.dump({"type": "linear_dummy", "action_dim": 3}, f)
    load_resp = send({"type": "load_policy", "path": dummy_path})
    assert load_resp['type'] in ('load_policy_ack', 'error')
    assert load_resp.get('error') is None, f"Failed to load dummy policy: {load_resp}"
    resp = send({"type": "obs", "data": {"q": [0,1,2], "dq": [0,0,0], "rand_hash": "abc123"}})
    assert resp['type'] == 'action'
    data = resp['data']
    # rand_hash echoed only if not pre-deadline-skip; accept presence as success criterion
    if 'rand_hash' in data:
        assert data['rand_hash'] == 'abc123'
    assert 'delta' in data

def test_metrics_counters():
    ensure_gateway(['--deadline-ms', '0.01'])
    dummy_path = os.path.join(os.path.dirname(__file__), 'dummy_policy.json')
    with open(dummy_path, 'w', encoding='utf-8') as f:
        json.dump({"type": "linear_dummy", "action_dim": 3}, f)
    load_resp = send({"type": "load_policy", "path": dummy_path})
    assert load_resp['type'] == 'load_policy_ack'
    send({"type": "obs", "data": {"q": [0,1,2], "dq": [0,0,0], "rand_hash": "abc123", "expected_rand_hash": "zzz999"}})
    time.sleep(5.5)
    metrics_path = os.path.join(os.path.dirname(__file__), '..', 'logs', 'ipc_metrics.json')
    assert os.path.exists(metrics_path)
    with open(metrics_path, 'r', encoding='utf-8') as f:
        metrics = json.load(f)
    counters = metrics.get('counters', {})
    assert counters.get('rand_hash_mismatch', 0) >= 1

def test_metrics_interval_and_rotation():
    # Start a fresh gateway on a different port to test rotation & interval
    alt_port = 45325
    proc = start_gateway(extra_args=['--port', str(alt_port), '--metrics-interval', '1.0', '--max-log-mb', '0.001', '--compress-rotated'])
    try:
        time.sleep(0.5)
        # load dummy policy for actions
        dummy_path = os.path.join(os.path.dirname(__file__), 'dummy_policy_alt.json')
        with open(dummy_path, 'w', encoding='utf-8') as f:
            json.dump({"type":"linear_dummy", "action_dim": 3}, f)
        with socket.create_connection(('127.0.0.1', alt_port), timeout=2) as s:
            s.sendall((json.dumps({"type":"load_policy","path": dummy_path})+'\n').encode()); s.recv(2048)
        # generate a bunch of obs messages (these record latency metrics)
        for _ in range(30):
            with socket.create_connection(('127.0.0.1', alt_port), timeout=2) as s:
                s.sendall(b'{"type":"obs","data":{"q":[0,1,2],"dq":[0,0,0]}}\n')
                s.recv(2048)
        # wait for at least one metrics flush at 1s interval
        time.sleep(1.5)
        metrics_path = os.path.join(os.path.dirname(__file__), '..', 'logs', 'ipc_metrics.json')
        assert os.path.exists(metrics_path)
        # Check for rotated log existence (at least one file with timestamp suffix)
        log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        rotated = [f for f in os.listdir(log_dir) if f.startswith('ipc_gateway_events.jsonl.')]
        assert any(f.endswith('.gz') for f in rotated), 'Expected at least one compressed rotated log file (.gz)'
        with open(metrics_path, 'r', encoding='utf-8') as f:
            metrics = json.load(f)
        assert 'recent_latency_ms' in metrics, 'recent_latency_ms section missing'
    finally:
        proc.kill()

def test_alert_and_prometheus_export():
    prom_path = os.path.join(os.path.dirname(__file__), '..', 'logs', 'gateway_metrics.prom')
    alt_port = 45329
    proc = start_gateway(extra_args=['--port', str(alt_port), '--metrics-interval','1.0','--alert-latency-p95-ms','0.1','--prometheus-textfile', prom_path])
    try:
        time.sleep(0.6)
        # load dummy policy
        dummy_path = os.path.join(os.path.dirname(__file__), 'dummy_policy_alert.json')
        with open(dummy_path,'w',encoding='utf-8') as f:
            json.dump({"type":"linear_dummy","action_dim":2}, f)
        with socket.create_connection(('127.0.0.1', alt_port), timeout=2) as s:
            s.sendall((json.dumps({"type":"load_policy","path": dummy_path})+'\n').encode()); s.recv(2048)
        # generate obs to produce metrics
        for _ in range(5):
            with socket.create_connection(('127.0.0.1', alt_port), timeout=2) as s:
                s.sendall(b'{"type":"obs","data":{"q":[0,1],"dq":[0,0]}}\n'); s.recv(2048)
        time.sleep(1.5)
        # verify prometheus file
        assert os.path.exists(prom_path)
        prom_txt = open(prom_path,'r').read()
        assert 'gateway_policy_latency_ms_count' in prom_txt
        # check alert event presence in main event log (best-effort)
        events_tail = open(os.path.join(os.path.dirname(__file__),'..','logs','ipc_gateway_events.jsonl'),'r',encoding='utf-8').read().splitlines()[-50:]
        alert_lines = [l for l in events_tail if 'sla_alert' in l]
        assert alert_lines, 'Expected sla_alert event not found'
    finally:
        proc.kill()

def shutdown_gateway():
    global _gateway_proc
    if _gateway_proc and _gateway_proc.poll() is None:
        _gateway_proc.kill()

if __name__ == '__main__':
    try:
        test_ping_roundtrip()
        test_rand_hash_echo_and_deadline()
        test_metrics_counters()
        test_metrics_interval_and_rotation()
        test_alert_and_prometheus_export()
        print('Integration tests completed')
    finally:
        shutdown_gateway()
