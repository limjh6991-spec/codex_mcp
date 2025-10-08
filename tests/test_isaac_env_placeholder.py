import pytest

# Isaac API가 없는 환경에서는 스킵
try:
    from src.envs.isaac_roarm_env import ISAAC_AVAILABLE, IsaacRoArmM3Env
except Exception:  # pragma: no cover
    ISAAC_AVAILABLE = False

@pytest.mark.skipif(not ISAAC_AVAILABLE, reason="Isaac Sim API not available")
def test_isaac_env_reset():
    env = IsaacRoArmM3Env(stage_path="/World/roarm.usd", joint_config={"joints": []})
    obs, info = env.reset()
    assert obs is not None
