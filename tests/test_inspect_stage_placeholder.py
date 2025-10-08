import pytest
import os

# Isaac 환경 없으면 skip
try:
    import omni.isaac.kit  # type: ignore
    ISAAC_AVAILABLE = True
except Exception:  # pragma: no cover
    ISAAC_AVAILABLE = False

@pytest.mark.skipif(not ISAAC_AVAILABLE, reason="Isaac Sim API not available")
def test_inspect_stage_script_exists():
    assert os.path.isfile("sim/inspect_stage.py")
