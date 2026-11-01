"""Motion plan guards; no API calls or credentials needed."""
import pytest
from tb3_llm_controller.gemini_planner import validate_plan, extract_json


def test_append_stop():
    assert validate_plan([{'action': 'FORWARD', 'value': 0.2}])[-1] == {'action': 'STOP', 'value': 0}


@pytest.mark.parametrize('plan', [
    [], [{'action': 'FORWARD'}], [{'action': 'FORWARD', 'value': 0}],
    [{'action': 'FORWARD', 'value': 0.6}], [{'action': 'TURN_LEFT', 'value': 181}],
    [{'action': 'FORWARD', 'value': float('nan')}], [{'action': 'FORWARD', 'value': True}],
    [{'action': 'GO_TO_PLACE', 'target': 'window'}], [{'action': 'STOP', 'value': 1}],
    ['bad'], [{'action': 'STOP', 'value': 0}] * 11,
])
def test_reject_bad_plan(plan):
    with pytest.raises(ValueError):
        validate_plan(plan)


def test_json_fences():
    assert extract_json('```json\n[{"action":"STOP","value":0}]\n```') == [{'action': 'STOP', 'value': 0}]
