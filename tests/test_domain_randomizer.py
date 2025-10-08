from src.utils.domain_randomizer import DomainRandomizer
import tempfile
import os
import textwrap


def test_domain_randomizer_sample():
    yaml_content = textwrap.dedent(
        """
        physics:
          dt_jitter: [0.0, 0.001]
        robot:
          joint_friction_scale: [0.5, 1.5]
          motor_strength_scale: 1.0
        """
    )
    with tempfile.NamedTemporaryFile("w", delete=False) as fp:
        fp.write(yaml_content)
        fname = fp.name
    try:
        dr = DomainRandomizer.from_yaml(fname)
        s1 = dr.sample()
        s2 = dr.sample(force=True)
        assert s1 != s2  # force=True 재샘플 → 다름 기대
        assert 0.0 <= s1["physics"]["dt_jitter"] <= 0.001
        assert 0.5 <= s1["robot"]["joint_friction_scale"] <= 1.5
    finally:
        os.remove(fname)
