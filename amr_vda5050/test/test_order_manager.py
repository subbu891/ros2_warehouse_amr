import copy

import pytest

from amr_vda5050 import order_builder as ob
from amr_vda5050.order_manager import OrderManager, topic

STATIONS = {"home": {"x": 2.0, "y": 4.0, "yaw": 0.0},
            "pickup_a": {"x": 2.5, "y": 8.0, "yaw": 0.0},
            "drop_b": {"x": 14.0, "y": 5.5, "yaw": 0.0}}
HOME = (2.0, 4.0)


@pytest.fixture
def om():
    return OrderManager("TUM", "amr_001")


def transport(order_id="o1"):
    return ob.transport_order(order_id, STATIONS, "home", "pickup_a", "drop_b")


def run_to_end(om, xy=HOME):
    done = []
    while (task := om.next_task(xy)) is not None:
        om.task_started()
        done.append(task)
        om.task_succeeded()
    return done


def test_topic_layout():
    assert topic("uagv", "v2", "TUM", "amr_001", "order") == "uagv/v2/TUM/amr_001/order"


def test_valid_order_is_accepted(om):
    assert om.receive_order(transport()) == []
    assert om.order_id == "o1" and len(om.node_states) == 3 and len(om.edge_states) == 2


@pytest.mark.parametrize("mutate, expected", [
    (lambda o: o.pop("nodes"), "missing field"),
    (lambda o: o["nodes"][1].update(sequenceId=3), "bad sequenceId"),
    (lambda o: o["edges"][0].update(endNodeId="x"), "does not connect"),
    (lambda o: o["nodes"][0].update(released=False), "first node must be released"),
    (lambda o: (o["nodes"][1].update(released=False)), "must precede horizon"),
    (lambda o: o["nodes"][1]["actions"].append(ob.action("fly")), "unsupported action"),
])
def test_invalid_orders_are_rejected(om, mutate, expected):
    o = transport()
    mutate(o)
    errs = om.receive_order(o)
    assert errs and any(expected in e["errorDescription"] for e in errs)
    # Nothing was loaded
    assert om.order_id == ""
    assert om.build_state(None, (0, 0), False, 90, False)["errors"]


def test_first_node_at_robot_position_is_reached_without_driving(om):
    om.receive_order(transport())
    task = om.next_task(HOME)
    # Home was skipped
    assert task.node_id == "pickup_a"
    assert om.last_node_id == "home"
    assert task.steps[0].startswith("goto_pose:2.5,8.0") and task.steps[1] == "wait:3.0"


def test_full_order_execution_updates_states(om):
    om.receive_order(transport())
    tasks = run_to_end(om)
    assert [t.node_id for t in tasks] == ["pickup_a", "drop_b"]
    s = om.build_state((14.0, 5.5, 0.0), (0, 0), False, 80, False)
    assert s["lastNodeId"] == "drop_b" and s["lastNodeSequenceId"] == 4
    assert s["nodeStates"] == [] and s["edgeStates"] == []
    assert {a["actionStatus"] for a in s["actionStates"]} == {"FINISHED"}
    assert not om.has_active_order


def test_edges_are_removed_when_their_end_node_is_reached(om):
    om.receive_order(transport())
    # Home reached, executing pickup_a
    om.next_task(HOME)
    assert [e["sequenceId"] for e in om.edge_states] == [1, 3]
    om.task_succeeded()
    assert [e["sequenceId"] for e in om.edge_states] == [3]


def test_new_order_rejected_while_active_then_accepted_after_finish(om):
    om.receive_order(transport("o1"))
    errs = om.receive_order(transport("o2"))
    assert errs and errs[0]["errorType"] == "orderError"
    run_to_end(om)
    assert om.receive_order(transport("o2")) == []


def test_order_update_extends_the_base(om):
    nodes = [ob.node("home", 0, 2.0, 4.0, 0.0), ob.node("pickup_a", 2, 2.5, 8.0, 0.0),
             ob.node("drop_b", 4, 14.0, 5.5, 0.0, released=False)]
    om.receive_order(ob.order("o1", nodes))
    run_to_end(om)
    assert om.last_node_id == "pickup_a"
    # Horizon only: wait for update
    assert om.next_task(HOME) is None
    upd = ob.order("o1", [ob.node("pickup_a", 2, 2.5, 8.0, 0.0), ob.node("drop_b", 4, 14.0, 5.5, 0.0)],
                   update_id=1)
    assert om.receive_order(upd) == []
    assert [t.node_id for t in run_to_end(om)] == ["drop_b"]


def test_order_update_with_wrong_stitching_node_or_old_id(om):
    om.receive_order(transport())
    # The last base node is drop_b (seq 4); stitching at pickup_a (seq 2) is invalid
    bad = ob.order("o1", [ob.node("pickup_a", 2, 2.5, 8.0), ob.node("home", 4, 2.0, 4.0)], update_id=1)
    assert om.receive_order(bad)[0]["errorType"] == "orderUpdateError"
    good = ob.order("o1", [ob.node("drop_b", 4, 14.0, 5.5), ob.node("home", 6, 2.0, 4.0)], update_id=1)
    assert om.receive_order(good) == []
    assert [n["nodeId"] for n in om.node_states] == ["home", "pickup_a", "drop_b", "home"]
    om.order_update_id = 5
    old = copy.deepcopy(transport())
    old["orderUpdateId"] = 2
    assert om.receive_order(old)[0]["errorType"] == "orderUpdateError"


def test_duplicate_update_is_ignored(om):
    om.receive_order(transport())
    assert om.receive_order(transport()) == []
    assert len(om.node_states) == 3


def test_failure_sets_fatal_error_and_cancel_clears_it(om):
    om.receive_order(transport())
    om.next_task(HOME)
    om.task_failed("goal blocked")
    s = om.build_state(None, (0, 0), False, 90, False)
    assert s["errors"][-1]["errorLevel"] == "FATAL"
    assert om.next_task(HOME) is None
    todo = om.instant_actions(ob.cancel_order())
    # Nothing executing any more
    assert todo == []
    assert om.node_states == [] and not any(e["errorType"] == "navigationError" for e in om.errors)
    assert om.receive_order(transport("o2")) == []


def test_cancel_while_executing_asks_adapter_to_cancel(om):
    om.receive_order(transport())
    om.next_task(HOME)
    assert om.instant_actions(ob.cancel_order()) == ["cancel"]
    assert om.executing is None


def test_cancel_without_order_fails(om):
    om.instant_actions(ob.cancel_order())
    assert om.action_states[-1]["actionStatus"] == "FAILED"
    assert om.errors[-1]["errorType"] == "noOrderToCancel"


def test_charge_order_docks_instead_of_driving(om):
    om.receive_order(ob.charge_order("c1", STATIONS, "home", (0.75, 2.0)))
    task = om.next_task(HOME)
    assert task.steps == ["dock:charger"]


def test_state_message_contains_mandatory_fields(om):
    s = om.build_state((1.0, 2.0, 0.5), (0.3, 0.1), True, 77.7, False, estop="NONE")
    for key in ("headerId", "timestamp", "version", "manufacturer", "serialNumber", "orderId",
                "orderUpdateId", "lastNodeId", "lastNodeSequenceId", "nodeStates", "edgeStates",
                "driving", "actionStates", "batteryState", "operatingMode", "errors", "safetyState"):
        assert key in s
    assert s["agvPosition"]["mapId"] == "warehouse"
    assert om.build_state(None, (0, 0), False, 0, False)["headerId"] == s["headerId"] + 1
