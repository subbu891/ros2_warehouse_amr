from __future__ import annotations

import uuid


def node(node_id: str, seq: int, x: float, y: float, theta: float | None = None,
         released: bool = True, actions: list[dict] | None = None, map_id: str = "warehouse") -> dict:
    pos = {"x": x, "y": y, "mapId": map_id, "allowedDeviationXY": 0.3}
    if theta is not None:
        pos["theta"] = theta
    return {"nodeId": node_id, "sequenceId": seq, "released": released,
            "nodePosition": pos, "actions": actions or []}


def edge(start: dict, end: dict, released: bool = True) -> dict:
    return {"edgeId": f"{start['nodeId']}->{end['nodeId']}", "sequenceId": start["sequenceId"] + 1,
            "released": released, "startNodeId": start["nodeId"], "endNodeId": end["nodeId"],
            "actions": []}


def action(action_type: str, blocking: str = "HARD", **params) -> dict:
    return {"actionId": f"{action_type}-{uuid.uuid4().hex[:8]}", "actionType": action_type,
            "blockingType": blocking,
            "actionParameters": [{"key": k, "value": v} for k, v in params.items()]}


def order(order_id: str, nodes: list[dict], update_id: int = 0, header: dict | None = None) -> dict:
    edges = [edge(a, b, released=b["released"]) for a, b in zip(nodes, nodes[1:])]
    msg = dict(header or {})
    msg.update({"orderId": order_id, "orderUpdateId": update_id, "nodes": nodes, "edges": edges})
    return msg


# Start -> pickup (pick) -> drop (drop). `stations`: name -> {x, y, yaw}
def transport_order(order_id: str, stations: dict, start: str, pickup: str, drop: str,
                    first_seq: int = 0) -> dict:
    names = [start, pickup, drop]
    acts = [[], [action("pick")], [action("drop")]]
    nodes = [node(n, first_seq + 2 * i, stations[n]["x"], stations[n]["y"], stations[n]["yaw"], actions=a)
             for i, (n, a) in enumerate(zip(names, acts))]
    return order(order_id, nodes)


def charge_order(order_id: str, stations: dict, start: str, dock_xy: tuple[float, float],
                 dock_id: str = "charger") -> dict:
    s = stations[start]
    nodes = [node(start, 0, s["x"], s["y"], s["yaw"]),
             node(f"dock_{dock_id}", 2, dock_xy[0], dock_xy[1],
                  actions=[action("startCharging", dockId=dock_id)])]
    return order(order_id, nodes)


def cancel_order() -> dict:
    return {"actions": [{"actionId": f"cancel-{uuid.uuid4().hex[:8]}", "actionType": "cancelOrder",
                         "blockingType": "HARD", "actionParameters": []}]}
