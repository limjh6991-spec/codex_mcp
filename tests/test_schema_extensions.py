import json, os

def test_schema_has_joint_names_and_action_scale_hint():
    path = os.path.join('configs','schemas','obs_action_schema.json')
    with open(path,'r',encoding='utf-8') as f:
        schema = json.load(f)
    obs_props = schema['properties']['observation']['properties']
    assert 'joint_names' in obs_props
    action_props = schema['properties']['action']['properties']
    assert 'action_scale_hint' in action_props
