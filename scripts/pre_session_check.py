#!/usr/bin/env python3
"""Pre-session automated checklist runner.

Performs:
1. Python version check
2. Launch ipc_policy_gateway subprocess
3. Ping + obs probes
4. Load dummy linear policy
5. Optional: run targeted pytest (deadline escalation)
6. Summarize results

Exit codes:
 0 = all core checks passed
 1 = gateway failed to start
 2 = probes failed
 3 = dummy policy load failed
 4 = tests failed (if tests requested)
"""
from __future__ import annotations
import argparse, subprocess, sys, time, socket, json, os, pathlib, textwrap, shlex, signal

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'
GW_SCRIPT = SCRIPTS / 'ipc_policy_gateway.py'
LOG_DIR_DEFAULT = pathlib.Path('/tmp/gw_check')
HOST='127.0.0.1'
PORT=45123

class Section:
    def __init__(self, name: str):
        self.name = name
        self.ok = True
        self.messages: list[str] = []
    def fail(self, msg: str):
        self.ok = False
        self.messages.append(msg)
    def info(self, msg: str):
        self.messages.append(msg)


def wait_port(host: str, port: int, timeout: float = 3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.05)
    return False


def send_recv(obj: dict, host=HOST, port=PORT, timeout=1.0):
    s = socket.create_connection((host, port), timeout=timeout)
    s.sendall((json.dumps(obj)+'\n').encode())
    data = s.recv(65536).decode().strip()
    s.close()
    return data


def run_pytest(expr: str):
    cmd = [sys.executable, '-m', 'pytest', '-k', expr, '-q']
    return subprocess.run(cmd, cwd=ROOT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skip-tests', action='store_true', help='Skip pytest invocation')
    ap.add_argument('--deadline-test-expr', default='deadline_escalation', help='Pytest -k expression for targeted test')
    ap.add_argument('--log-dir', default=str(LOG_DIR_DEFAULT))
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--host', default=HOST)
    ap.add_argument('--policy-action-dim', type=int, default=3)
    ap.add_argument('--keep-gateway', action='store_true', help='Do not terminate gateway at end')
    args = ap.parse_args()

    # Section objects
    s_env = Section('environment')
    s_gateway = Section('gateway')
    s_probe = Section('probe')
    s_policy = Section('policy')
    s_tests = Section('tests')

    py_ver = sys.version.split()[0]
    s_env.info(f'python_version={py_ver}')
    if not GW_SCRIPT.exists():
        s_gateway.fail(f'gateway script missing: {GW_SCRIPT}')
        summarize([s_env, s_gateway, s_probe, s_policy, s_tests])
        return 1

    log_dir = pathlib.Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    gw_cmd = [sys.executable, str(GW_SCRIPT), '--port', str(args.port), '--deadline-ms', '1.0', '--deadline-escalate-threshold', '3', '--degrade-mode-ratio', '0.5', '--log-dir', str(log_dir)]
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    gw_proc = subprocess.Popen(gw_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)

    start_deadline = time.time() + 5.0
    started = False
    gateway_lines: list[str] = []
    while time.time() < start_deadline:
        line = gw_proc.stdout.readline()
        if not line:
            time.sleep(0.05)
            if gw_proc.poll() is not None:
                s_gateway.fail(f'gateway exited early code={gw_proc.returncode}')
                break
            continue
        gateway_lines.append(line.rstrip())
        if 'listening on' in line:
            started = True
            break
    if not started:
        s_gateway.fail('did not see listening banner')
    else:
        s_gateway.info('startup banner detected')

    if started and not wait_port(args.host, args.port, 2.0):
        s_gateway.fail('port not accepting connections')

    # Probes
    if s_gateway.ok:
        try:
            pong = send_recv({'type':'ping'}, host=args.host, port=args.port)
            s_probe.info(f'ping_reply={pong}')
            obs_reply = send_recv({'type':'obs','data':{'q':[0,1,2],'dq':[0,0,0]}}, host=args.host, port=args.port)
            s_probe.info(f'obs_reply={obs_reply}')
        except Exception as e:
            s_probe.fail(f'probe error: {e}')

    # Dummy policy load
    if s_gateway.ok and s_probe.ok:
        try:
            dummy_path = log_dir / 'dummy_policy.json'
            dummy_path.write_text(json.dumps({'type':'linear_dummy','action_dim':args.policy_action_dim}))
            load_reply = send_recv({'type':'load_policy','path':str(dummy_path)}, host=args.host, port=args.port)
            s_policy.info(f'load_reply={load_reply}')
            if 'error' in load_reply.lower():
                s_policy.fail('policy load returned error')
        except Exception as e:
            s_policy.fail(f'policy load exception: {e}')

    # Tests (optional)
    if not args.skip_tests and s_gateway.ok and s_probe.ok:
        r = run_pytest(args.deadline_test_expr)
        if r.returncode != 0:
            s_tests.fail(f'pytest failed rc={r.returncode}')
        else:
            s_tests.info('pytest targeted tests passed')

    if not args.keep_gateway and gw_proc.poll() is None:
        try:
            gw_proc.send_signal(signal.SIGINT)
            try:
                gw_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                gw_proc.kill()
        except Exception:
            gw_proc.kill()

    sections = [s_env, s_gateway, s_probe, s_policy, s_tests]
    summarize(sections)

    # exit code precedence
    if not s_gateway.ok:
        return 1
    if not s_probe.ok:
        return 2
    if not s_policy.ok:
        return 3
    if not args.skip_tests and not s_tests.ok:
        return 4
    return 0


def summarize(sections):
    print('\n=== PRE-SESSION CHECK SUMMARY ===')
    for sec in sections:
        status = 'OK' if sec.ok else 'FAIL'
        print(f'[{status}] {sec.name}')
        for m in sec.messages:
            print(f'  - {m}')
    print('=================================')

if __name__ == '__main__':
    rc = main()
    sys.exit(rc)
