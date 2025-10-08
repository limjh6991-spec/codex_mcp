from src.utils.physics_randomizer import apply_physics_randomization

class DummyWorld:
    def get_physics_context(self):
        class Ctx:
            def set_solver_position_iteration_count(self, v: int):
                self.pos_it = v
            def set_solver_velocity_iteration_count(self, v: int):
                self.vel_it = v
        return Ctx()

def test_physics_randomizer_report_structure():
    sample = {
        'physics': {'solver_position_iterations': 6, 'solver_velocity_iterations': 2},
        'robot': {'joint_damping_scale': 1.1},
        'environment': {'ground_friction': 0.8}
    }
    report = apply_physics_randomization(DummyWorld(), sample)
    assert report['applied_physics'] is True
    assert report['applied_robot'] is True
    assert report['applied_env'] is True
    assert 'physics_applied_detail' in report
    assert 'solver_position_iterations' in report['physics_applied_detail']
    # Basic structural keys
    for key in ['success_keys', 'failed_keys', 'skipped_keys', 'value_map', 'ranges']:
        assert key in report
    # Damping scale will likely be skipped due to no articulation in DummyWorld
    assert 'joint_damping_scale' in report['robot_applied_detail']
    # Ground friction attempt either success or skipped
    assert 'ground_friction' in report['env_applied_detail']
