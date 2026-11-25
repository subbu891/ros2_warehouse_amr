from __future__ import annotations

import argparse
import json
import random
import threading
import time
import uuid
from pathlib import Path

import yaml

from amr_vda5050 import order_builder as ob
from amr_vda5050.mqtt_compat import make_client
from amr_vda5050.order_manager import topic


def default_stations_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory("amr_mission")) / "config" / "stations.yaml")
    except (ImportError, LookupError):
        # Running outside a ROS workspace: use the source tree instead
        return str(Path(__file__).resolve().parents[2] / "amr_mission" / "config" / "stations.yaml")


class FleetManager:
    def __init__(self, host, port, manufacturer, serial, interface="uagv", major="v2"):
        self.t = {k: topic(interface, major, manufacturer, serial, k)
                  for k in ("order", "instantActions", "state", "connection")}
        self.header = {"version": "2.0.0", "manufacturer": manufacturer, "serialNumber": serial}
        self.header_id = 0
        self.state: dict | None = None
        self.state_event = threading.Event()
        self.client = make_client(f"fleet-{uuid.uuid4().hex[:6]}")
        self.client.on_message = self._on_message
        self.client.connect(host, port, keepalive=30)
        self.client.subscribe([(self.t["state"], 0), (self.t["connection"], 1)])
        self.client.loop_start()

    def _on_message(self, _c, _u, msg):
        data = json.loads(msg.payload)
        if msg.topic == self.t["state"]:
            self.state = data
            self.state_event.set()
        else:
            print(f"[connection] {data.get('connectionState')}")

    def _send(self, kind: str, msg: dict):
        msg.update(self.header)
        msg["headerId"] = self.header_id
        msg["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())
        self.header_id += 1
        self.client.publish(self.t[kind], json.dumps(msg), qos=1)

    def send_order(self, order: dict):
        print(f"-> order {order['orderId']}: {[n['nodeId'] for n in order['nodes']]}")
        self._send("order", order)

    def cancel(self):
        print("-> instantAction cancelOrder")
        self._send("instantActions", ob.cancel_order())

    def wait_state(self, timeout=10.0) -> dict | None:
        self.state_event.clear()
        self.state_event.wait(timeout)
        return self.state

    def nearest_station(self, stations: dict) -> str:
        st = self.wait_state() or {}
        pos = st.get("agvPosition")
        if not pos:
            return "home"
        return min(stations, key=lambda n: (stations[n]["x"] - pos["x"]) ** 2 + (stations[n]["y"] - pos["y"]) ** 2)

    def wait_order_done(self, order_id: str, timeout: float) -> tuple[bool, str]:
        start = time.time()
        while time.time() - start < timeout:
            st = self.wait_state(2.0)
            if not st or st.get("orderId") != order_id:
                continue
            fatal = [e for e in st.get("errors", []) if e.get("errorLevel") == "FATAL"]
            if fatal:
                return False, fatal[0]["errorDescription"]
            rejected = [e for e in st.get("errors", []) if e["errorType"] in ("validationError", "orderError")]
            if rejected:
                return False, rejected[0]["errorDescription"]
            if not st["nodeStates"] and st["lastNodeId"]:
                return True, f"done in {time.time() - start:.1f} s"
        return False, "timeout"


def print_state(st: dict):
    pos = st.get("agvPosition", {})
    print(f"order={st['orderId']}/{st['orderUpdateId']} last={st['lastNodeId']} "
          f"pending={[n['nodeId'] for n in st['nodeStates']]} driving={st['driving']} "
          f"pos=({pos.get('x', 0):.2f},{pos.get('y', 0):.2f}) battery={st['batteryState']['batteryCharge']}% "
          f"safety={st['safetyState']} errors={[e['errorType'] for e in st['errors']]}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Minimal VDA 5050 fleet manager for testing the AMR over MQTT")
    ap.add_argument("command", choices=["transport", "charge", "cancel", "monitor", "endurance"])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--manufacturer", default="TUM-Student")
    ap.add_argument("--serial", default="amr_001")
    ap.add_argument("--stations", default=default_stations_file())
    ap.add_argument("--pickup", default="pickup_a")
    ap.add_argument("--drop", default="drop_b")
    ap.add_argument("--dock-xy", type=float, nargs=2, default=[0.75, 2.0])
    ap.add_argument("--jobs", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--seed", type=int, default=0)
    args, _ = ap.parse_known_args(argv)

    stations = yaml.safe_load(Path(args.stations).read_text())["stations"]
    fm = FleetManager(args.host, args.port, args.manufacturer, args.serial)

    if args.command == "monitor":
        try:
            while True:
                st = fm.wait_state(5.0)
                if st:
                    print_state(st)
        except KeyboardInterrupt:
            return 0
    if args.command == "cancel":
        fm.cancel()
        time.sleep(1.0)
        return 0
    if args.command == "charge":
        oid = f"charge-{uuid.uuid4().hex[:6]}"
        fm.send_order(ob.charge_order(oid, stations, fm.nearest_station(stations), tuple(args.dock_xy)))
        ok, msg = fm.wait_order_done(oid, args.timeout)
        print(("OK " if ok else "FAILED ") + msg)
        return 0 if ok else 1
    if args.command == "transport":
        oid = f"transport-{uuid.uuid4().hex[:6]}"
        fm.send_order(ob.transport_order(oid, stations, fm.nearest_station(stations), args.pickup, args.drop))
        ok, msg = fm.wait_order_done(oid, args.timeout)
        print(("OK " if ok else "FAILED ") + msg)
        return 0 if ok else 1

    # Endurance run: random transport jobs between stations
    rng = random.Random(args.seed)
    names = sorted(stations)
    results = []
    for i in range(args.jobs):
        start = fm.nearest_station(stations)
        pickup, drop = rng.sample([n for n in names if n != start], 2)
        oid = f"endurance-{i:03d}"
        fm.send_order(ob.transport_order(oid, stations, start, pickup, drop))
        ok, msg = fm.wait_order_done(oid, args.timeout)
        results.append(ok)
        print(f"[{i + 1}/{args.jobs}] {pickup} -> {drop}: {'OK' if ok else 'FAILED'} ({msg})")
        if not ok:
            fm.cancel()
            time.sleep(2.0)
    rate = 100.0 * sum(results) / max(len(results), 1)
    print(f"\nEndurance result: {sum(results)}/{len(results)} jobs succeeded ({rate:.1f} %)")
    return 0 if rate >= 95.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
