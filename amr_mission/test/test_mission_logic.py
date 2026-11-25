import pytest

from amr_mission.mission_logic import (MissionError, MissionRunner, Outcome, Pose2D, RetryPolicy,
                                       StepKind, load_stations, parse_mission, parse_step)

STATIONS = {"pickup_a": Pose2D(2.5, 8.0, 0.0), "drop_b": Pose2D(14.0, 5.5, 0.0)}


# Fake robot: returns scripted outcomes and records every call
class FakeExecutor:
    def __init__(self, script=None):
        self.script = list(script or [])
        self.calls = []
        self.recoveries = []

    def _next(self, call):
        self.calls.append(call)
        return self.script.pop(0) if self.script else Outcome.SUCCEEDED

    def navigate(self, pose):
        return self._next(("navigate", pose))

    def dock(self, dock_id):
        return self._next(("dock", dock_id))

    def undock(self):
        return self._next(("undock",))

    def wait(self, seconds):
        return self._next(("wait", seconds))

    def recover(self, attempt):
        self.recoveries.append(attempt)


def run(steps, script=None, attempts=3):
    ex = FakeExecutor(script)
    res = MissionRunner(ex, RetryPolicy(attempts), clock=lambda: 0.0).run(parse_mission(steps, STATIONS))
    return res, ex


def test_parse_all_step_kinds():
    assert parse_step("goto:pickup_a", STATIONS).pose == STATIONS["pickup_a"]
    assert parse_step("goto_pose: 1.0, 2.0, 0.5", STATIONS).pose == Pose2D(1.0, 2.0, 0.5)
    assert parse_step("wait:2.5", STATIONS).seconds == 2.5
    assert parse_step("dock:charger", STATIONS).target == "charger"
    assert parse_step("UNDOCK", STATIONS).kind == StepKind.UNDOCK


@pytest.mark.parametrize("bad", ["goto:nowhere", "goto_pose:1,2", "wait:abc", "wait:-1", "dock:", "fly:moon"])
def test_parse_rejects_bad_steps(bad):
    with pytest.raises(MissionError):
        parse_step(bad, STATIONS)


def test_cannot_drive_while_docked():
    with pytest.raises(MissionError):
        parse_mission(["dock:charger", "goto:drop_b"], STATIONS)
    parse_mission(["dock:charger", "undock", "goto:drop_b"], STATIONS)


def test_empty_mission_rejected():
    with pytest.raises(MissionError):
        parse_mission([], STATIONS)


def test_happy_path_executes_steps_in_order():
    res, ex = run(["goto:pickup_a", "wait:3", "goto:drop_b", "dock:charger"])
    assert res.success and res.failed_step == -1
    assert [c[0] for c in ex.calls] == ["navigate", "wait", "navigate", "dock"]
    assert ex.recoveries == []


def test_failed_navigation_is_retried_after_recovery():
    res, ex = run(["goto:pickup_a", "goto:drop_b"], script=[Outcome.FAILED, Outcome.SUCCEEDED])
    assert res.success
    assert ex.recoveries == [1]
    assert res.attempts == [2, 1]


def test_gives_up_after_max_attempts():
    res, ex = run(["goto:pickup_a", "goto:drop_b"], script=[Outcome.FAILED] * 3, attempts=3)
    assert not res.success and res.failed_step == 0
    assert ex.recoveries == [1, 2]
    # Step 1 never started
    assert len(ex.calls) == 3


def test_cancel_stops_without_retry():
    res, ex = run(["goto:pickup_a", "goto:drop_b"], script=[Outcome.CANCELED])
    assert not res.success and res.message == "canceled"
    assert ex.recoveries == []


def test_feedback_reports_status():
    events = []
    ex = FakeExecutor([Outcome.FAILED])
    MissionRunner(ex, clock=lambda: 0.0).run(parse_mission(["goto:pickup_a"], STATIONS),
                                             lambda i, s, st, a: events.append((i, st, a)))
    assert events == [(0, "RUNNING", 1), (0, "RECOVERING", 1), (0, "RETRYING", 2)]


def test_load_stations():
    st = load_stations({"stations": {"home": {"x": 1, "y": 2, "yaw": 0.5}}})
    assert st["home"] == Pose2D(1.0, 2.0, 0.5)
