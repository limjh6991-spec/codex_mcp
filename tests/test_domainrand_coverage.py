from src.utils.domain_randomizer import DomainRandomizer
from src.utils.domainrand_coverage import analyze_samples, report_to_markdown
import json

CONFIG = {
    "physics": {
        "dt_jitter": [0.0, 0.0005],
        "solver_position_iteration_range": [4, 8],
    },
    "robot": {"joint_friction_scale": [0.8, 1.2]},
}

def test_domain_randomization_coverage(tmp_path):
    log_file = tmp_path / "rand_samples.jsonl"
    dr = DomainRandomizer(CONFIG, log_path=str(log_file))
    # generate samples
    samples = []
    for _ in range(20):
        samples.append(dr.sample(force=True))
    # read back from log
    logged = []
    for line in log_file.read_text(encoding="utf-8").splitlines():
        logged.append(json.loads(line))
    assert len(logged) == 20
    rep = analyze_samples(logged, CONFIG)
    assert rep.total_samples == 20
    # ensure stats present for expected flattened keys
    flat_keys = [
        "physics.dt_jitter",
        "physics.solver_position_iteration_range",
        "robot.joint_friction_scale",
    ]
    for k in flat_keys:
        assert k in rep.flat_stats, f"Missing stats for {k}"
    # markdown smoke
    md = report_to_markdown(rep)
    assert "Domain Randomization Coverage" in md
