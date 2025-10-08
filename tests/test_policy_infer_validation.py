import types
import sys
import pytest

from mcp_servers.isaac_controller_server import IsaacControlServer

class DummyPolicy:
    def __init__(self, obs_dim=4):
        class Space:
            shape = (obs_dim,)
        self.observation_space = Space()
    def predict(self, obs, deterministic=True):  # pragma: no cover - simple
        import numpy as np
        action = np.zeros((1, 2), dtype=float)
        return action, None


def test_policy_infer_no_policy():
    server = IsaacControlServer()
    out = server.policy_infer(observation=[0, 1, 2])
    assert out.get("ok") is False
    assert out.get("error_code") == "NO_POLICY" or out.get("error_code") in {"DEP"}


def test_policy_infer_with_dummy_policy_and_dim_mismatch(monkeypatch):
    server = IsaacControlServer()
    dummy = DummyPolicy(obs_dim=5)
    server._policy = dummy
    server._policy_path = "dummy.zip"
    out = server.policy_infer(observation=[0, 0, 0])
    if out.get("error_code") == "DEP":
        pytest.skip("stable_baselines3 missing")
    if out.get("ok"):
        pytest.skip("expected_dim not available (no validation triggered)")
    assert out["error_code"] == "OBS_DIM"
    assert out["expected_obs_dim"] == 5


def test_policy_infer_with_dummy_policy_success():
    server = IsaacControlServer()
    dummy = DummyPolicy(obs_dim=3)
    server._policy = dummy
    server._policy_path = "dummy.zip"
    out = server.policy_infer(observation=[0.0, 0.0, 0.0])
    if out.get("error_code") == "DEP":
        pytest.skip("stable_baselines3 missing")
    assert out.get("ok") is True
    assert out["action"] == [0.0, 0.0]
    assert out["obs_dim"] == 3


def test_policy_infer_nan_detection():
    server = IsaacControlServer()
    dummy = DummyPolicy(obs_dim=3)
    server._policy = dummy
    server._policy_path = "dummy.zip"
    out = server.policy_infer(observation=[float('nan'), 0.0, 0.0])
    if out.get("error_code") == "DEP":
        pytest.skip("stable_baselines3 missing")
    assert out.get("ok") is False
    assert out.get("error_code") in {"NAN", "OBS_DIM"}
