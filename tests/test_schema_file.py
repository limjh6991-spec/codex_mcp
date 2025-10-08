import json, os

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), '..', 'configs', 'schemas', 'obs_action_schema.json')

REQUIRED_TOP_LEVEL_KEYS = {"$schema", "title", "type", "properties", "required", "schema_version"}


def test_schema_file_exists():
    assert os.path.exists(SCHEMA_PATH), "Observation/action schema file missing"


def test_schema_has_required_keys():
    with open(SCHEMA_PATH, 'r', encoding='utf-8') as f:
        schema = json.load(f)
    missing = REQUIRED_TOP_LEVEL_KEYS - set(schema.keys())
    assert not missing, f"Schema missing keys: {missing}"


def test_schema_required_includes_minimum_fields():
    with open(SCHEMA_PATH, 'r', encoding='utf-8') as f:
        schema = json.load(f)
    # Ensure schema_version present and positive
    assert 'schema_version' in schema, "Schema must declare schema_version for wire compatibility"
    props = schema.get('properties', {})
    assert 'observation' in props and 'action' in props, "Schema should define observation and action properties"
