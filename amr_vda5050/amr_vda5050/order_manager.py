from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone

VDA_VERSION = "2.0.0"
SUPPORTED_NODE_ACTIONS = {"pick", "drop", "startCharging", "stopCharging"}
SUPPORTED_INSTANT_ACTIONS = {"cancelOrder"}


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def topic(interface: str, major: str, manufacturer: str, serial: str, kind: str) -> str:
    return f"{interface}/{major}/{manufacturer}/{serial}/{kind}"


def make_error(error_type: str, description: str, level: str = "WARNING", refs=None) -> dict:
    return {"errorType": error_type, "errorLevel": level, "errorDescription": description,
            "errorReferences": refs or []}


# One unit of work for the mission executor: reach one node and run its actions
@dataclass
class Task:
    node_id: str
    sequence_id: int
    steps: list[str]
    action_ids: list[str]


@dataclass
class OrderManager:
    manufacturer: str
    serial_number: str
    map_id: str = "warehouse"
    default_dock: str = "charger"
    # Simulated pick/drop duration [s]
    handling_time: float = 3.0

    order_id: str = ""
    order_update_id: int = 0
    last_node_id: str = ""
    last_node_sequence_id: int = 0
    # Not yet traversed nodes (incl. horizon)
    node_states: list[dict] = field(default_factory=list)
    edge_states: list[dict] = field(default_factory=list)
    action_states: list[dict] = field(default_factory=list)
    errors: list[dict] = field(default_factory=list)
    _executing: Task | None = None
    _failed: bool = False
    _header_ids: itertools.count = field(default_factory=lambda: itertools.count())

    # Validation
    @staticmethod
    def validate(order: dict) -> list[dict]:
        errs = []
        for key in ("orderId", "orderUpdateId", "nodes", "edges"):
            if key not in order:
                errs.append(make_error("validationError", f"missing field '{key}'", "WARNING"))
        if errs:
            return errs
        nodes, edges = order["nodes"], order["edges"]
        if not nodes:
            return [make_error("validationError", "order has no nodes")]
        if len(edges) != len(nodes) - 1:
            errs.append(make_error("validationError", "need exactly len(nodes)-1 edges"))
        first = nodes[0]["sequenceId"]
        for i, n in enumerate(nodes):
            if n["sequenceId"] != first + 2 * i or n["sequenceId"] % 2:
                errs.append(make_error("validationError", f"node {n.get('nodeId')} has bad sequenceId"))
        for i, e in enumerate(edges):
            if e["sequenceId"] != first + 2 * i + 1:
                errs.append(make_error("validationError", f"edge {e.get('edgeId')} has bad sequenceId"))
            elif i < len(nodes) - 1 and (e["startNodeId"] != nodes[i]["nodeId"]
                                         or e["endNodeId"] != nodes[i + 1]["nodeId"]):
                errs.append(make_error("validationError", f"edge {e['edgeId']} does not connect its nodes"))
        node_rel = [n["released"] for n in nodes]
        if not node_rel[0]:
            errs.append(make_error("validationError", "first node must be released"))
        if node_rel != sorted(node_rel, reverse=True):
            errs.append(make_error("validationError", "released (base) nodes must precede horizon nodes"))
        for n in nodes:
            if n["released"] and "nodePosition" not in n:
                errs.append(make_error("validationError", f"node {n['nodeId']} needs nodePosition"))
            for a in n.get("actions", []):
                if a["actionType"] not in SUPPORTED_NODE_ACTIONS:
                    errs.append(make_error("validationError", f"unsupported action '{a['actionType']}'",
                                           refs=[{"referenceKey": "actionId", "referenceValue": a["actionId"]}]))
        for e in edges:
            for a in e.get("actions", []):
                errs.append(make_error("validationError", f"edge action '{a['actionType']}' not supported"))
        return errs

    # Orders
    @property
    def has_active_order(self) -> bool:
        return bool(self.node_states) and not self._failed

    # Returns errors (empty list = accepted). Errors are also kept for the state message
    def receive_order(self, order: dict, robot_xy: tuple[float, float] | None = None) -> list[dict]:
        errs = self.validate(order)
        if not errs:
            if order["orderId"] == self.order_id and self.order_id:
                errs = self._update(order)
            else:
                errs = self._new(order, robot_xy)
        self.errors = errs + [e for e in self.errors if e["errorType"] not in
                              ("validationError", "orderUpdateError", "orderError")]
        return errs

    def _new(self, order: dict, robot_xy) -> list[dict]:
        if self.has_active_order:
            return [make_error("orderError", f"order {self.order_id} still active; cancel it first")]
        self._failed = False
        self.order_id, self.order_update_id = order["orderId"], order["orderUpdateId"]
        self.errors = [e for e in self.errors if e["errorType"] != "navigationError"]
        self._load(order["nodes"], order["edges"], replace=True)
        return []

    def _update(self, order: dict) -> list[dict]:
        upd = order["orderUpdateId"]
        if upd < self.order_update_id:
            return [make_error("orderUpdateError", f"orderUpdateId {upd} < current {self.order_update_id}")]
        if upd == self.order_update_id:
            # Duplicate: ignore silently
            return []
        stitch = order["nodes"][0]
        base = [n for n in self.node_states if n["released"]]
        end_of_base = base[-1] if base else {"nodeId": self.last_node_id,
                                             "sequenceId": self.last_node_sequence_id}
        if (stitch["nodeId"], stitch["sequenceId"]) != (end_of_base["nodeId"], end_of_base["sequenceId"]):
            return [make_error("orderUpdateError", "update must start at the last base node")]
        self.order_update_id = upd
        # Keep our base, drop the old horizon, append the new part after the stitching node
        self.node_states = [n for n in self.node_states if n["released"]]
        self.edge_states = [e for e in self.edge_states if e["released"]]
        stitch_actions = stitch.get("actions", [])
        if stitch_actions and self.node_states:
            self.node_states[-1]["_actions"] = self.node_states[-1].get("_actions", []) + stitch_actions
            self._add_actions(stitch_actions)
        self._load(order["nodes"][1:], order["edges"], replace=False)
        return []

    def _add_actions(self, actions: list[dict]) -> None:
        for a in actions:
            self.action_states.append({"actionId": a["actionId"], "actionType": a["actionType"],
                                       "actionStatus": "WAITING", "resultDescription": ""})

    def _load(self, nodes: list[dict], edges: list[dict], replace: bool) -> None:
        if replace:
            self.node_states, self.edge_states, self.action_states = [], [], []
        for n in nodes:
            st = {"nodeId": n["nodeId"], "sequenceId": n["sequenceId"], "released": n["released"]}
            if "nodePosition" in n:
                st["nodePosition"] = n["nodePosition"]
            st["_actions"] = n.get("actions", [])
            self.node_states.append(st)
            self._add_actions(n.get("actions", []))
        for e in edges:
            self.edge_states.append({"edgeId": e["edgeId"], "sequenceId": e["sequenceId"],
                                     "released": e["released"]})

    # Execution
    def next_task(self, robot_xy: tuple[float, float] | None = None) -> Task | None:
        if self._executing or self._failed or not self.node_states:
            return None
        node = self.node_states[0]
        if not node["released"]:
            # Waiting for an order update
            return None
        pos = node["nodePosition"]
        steps: list[str] = []
        actions = node.get("_actions", [])
        docks = any(a["actionType"] == "startCharging" for a in actions)
        dev = pos.get("allowedDeviationXY", 0.3) or 0.3
        already_there = robot_xy is not None and math.dist(robot_xy, (pos["x"], pos["y"])) <= dev
        if not docks and not already_there:
            theta = pos.get("theta")
            if theta is None:
                theta = self._heading_to(node, robot_xy)
            steps.append(f"goto_pose:{pos['x']},{pos['y']},{theta}")
        for a in actions:
            steps.append(self._action_step(a))
        # Nothing to do: node reached immediately
        if not steps:
            self._reach(node)
            return self.next_task(robot_xy)
        task = Task(node["nodeId"], node["sequenceId"], steps, [a["actionId"] for a in actions])
        self._executing = task
        for a in self.action_states:
            if a["actionId"] in task.action_ids:
                a["actionStatus"] = "INITIALIZING"
        return task

    def _heading_to(self, node: dict, robot_xy) -> float:
        if robot_xy is None:
            return 0.0
        p = node["nodePosition"]
        return math.atan2(p["y"] - robot_xy[1], p["x"] - robot_xy[0])

    def _action_step(self, action: dict) -> str:
        params = {p["key"]: p["value"] for p in action.get("actionParameters", [])}
        t = action["actionType"]
        if t in ("pick", "drop"):
            return f"wait:{float(params.get('duration', self.handling_time))}"
        if t == "startCharging":
            return f"dock:{params.get('dockId', self.default_dock)}"
        # stopCharging
        return "undock"

    def task_started(self) -> None:
        if self._executing:
            for a in self.action_states:
                if a["actionId"] in self._executing.action_ids:
                    a["actionStatus"] = "RUNNING"

    def _reach(self, node: dict) -> None:
        self.last_node_id, self.last_node_sequence_id = node["nodeId"], node["sequenceId"]
        self.node_states.pop(0)
        # The edge leading to the next node stays until that node is reached
        self.edge_states = [e for e in self.edge_states if e["sequenceId"] > node["sequenceId"]]

    def task_succeeded(self) -> None:
        task, self._executing = self._executing, None
        if task is None:
            return
        for a in self.action_states:
            if a["actionId"] in task.action_ids:
                a["actionStatus"] = "FINISHED"
        self._reach(self.node_states[0])

    def task_failed(self, message: str) -> None:
        task, self._executing = self._executing, None
        if task is None:
            return
        self._failed = True
        for a in self.action_states:
            if a["actionId"] in task.action_ids:
                a["actionStatus"] = "FAILED"
                a["resultDescription"] = message
        self.errors.append(make_error("navigationError", message, "FATAL",
                                      [{"referenceKey": "nodeId", "referenceValue": task.node_id}]))

    @property
    def executing(self) -> Task | None:
        return self._executing

    # Instant actions
    # Handle instantActions; returns the list of things the adapter must do ('cancel')
    def instant_actions(self, msg: dict) -> list[str]:
        todo = []
        for a in msg.get("actions", msg.get("instantActions", [])):
            st = {"actionId": a["actionId"], "actionType": a["actionType"],
                  "actionStatus": "FINISHED", "resultDescription": ""}
            if a["actionType"] not in SUPPORTED_INSTANT_ACTIONS:
                st["actionStatus"] = "FAILED"
                st["resultDescription"] = "unsupported"
                self.errors.append(make_error("validationError", f"unsupported instant action {a['actionType']}"))
            elif a["actionType"] == "cancelOrder":
                if not self.node_states and not self._failed:
                    st["actionStatus"] = "FAILED"
                    self.errors.append(make_error("noOrderToCancel", "no active order"))
                else:
                    if self._executing:
                        todo.append("cancel")
                    self.cancel()
            self.action_states.append(st)
        return todo

    def cancel(self) -> None:
        for a in self.action_states:
            if a["actionStatus"] in ("WAITING", "INITIALIZING", "RUNNING"):
                a["actionStatus"] = "FAILED"
                a["resultDescription"] = "order canceled"
        self.node_states, self.edge_states = [], []
        self._executing = None
        self._failed = False
        self.errors = [e for e in self.errors if e["errorType"] != "navigationError"]

    # State message
    def header(self) -> dict:
        return {"headerId": next(self._header_ids), "timestamp": iso_now(), "version": VDA_VERSION,
                "manufacturer": self.manufacturer, "serialNumber": self.serial_number}

    def build_state(self, pose: tuple[float, float, float] | None, velocity: tuple[float, float],
                    driving: bool, battery: float, charging: bool, estop: str = "NONE",
                    field_violation: bool = False, localization_score: float | None = None) -> dict:
        state = self.header()
        state.update({
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.last_node_sequence_id,
            "driving": driving,
            "operatingMode": "AUTOMATIC",
            "nodeStates": [{k: v for k, v in n.items() if not k.startswith("_")} for n in self.node_states],
            "edgeStates": list(self.edge_states),
            "actionStates": list(self.action_states),
            "batteryState": {"batteryCharge": round(battery, 1), "charging": charging},
            "errors": list(self.errors),
            "information": [],
            "safetyState": {"eStop": estop, "fieldViolation": field_violation},
            "velocity": {"vx": velocity[0], "vy": 0.0, "omega": velocity[1]},
        })
        if pose is not None:
            state["agvPosition"] = {"x": pose[0], "y": pose[1], "theta": pose[2], "mapId": self.map_id,
                                    "positionInitialized": True}
            if localization_score is not None:
                state["agvPosition"]["localizationScore"] = localization_score
        return state

    def connection(self, status: str) -> dict:
        msg = self.header()
        # One of ONLINE, OFFLINE, CONNECTIONBROKEN
        msg["connectionState"] = status
        return msg
