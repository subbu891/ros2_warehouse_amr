import json
import sys
import threading
import time
from pathlib import Path

from amr_vda5050 import fleet_manager
from amr_vda5050.mqtt_compat import make_client
from amr_vda5050.order_manager import OrderManager, topic

REPO = Path(__file__).resolve().parents[1]
STATIONS = str(REPO / "amr_mission" / "config" / "stations.yaml")
TOPICS = {k: topic("uagv", "v2", "TUM-Student", "amr_001", k) for k in ("order", "instantActions", "state")}


# Simulated vehicle: real OrderManager logic behind MQTT, driving is instantaneous
class FakeRobot:
    def __init__(self):
        self.om = OrderManager("TUM-Student", "amr_001")
        self.pose = [2.0, 4.0, 0.0]
        self.lock = threading.Lock()
        self.client = make_client("fake-robot")
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883)
        self.client.subscribe([(TOPICS["order"], 1), (TOPICS["instantActions"], 1)])
        self.client.loop_start()
        threading.Thread(target=self.loop, daemon=True).start()

    def on_message(self, _client, _userdata, msg):
        with self.lock:
            data = json.loads(msg.payload)
            if msg.topic == TOPICS["order"]:
                errors = self.om.receive_order(data, tuple(self.pose[:2]))
                print("robot: order", data["orderId"], "->", errors or "accepted")
            else:
                self.om.instant_actions(data)

    # Execute every released node immediately and publish the state at 5 Hz
    def loop(self):
        while True:
            with self.lock:
                task = self.om.next_task(tuple(self.pose[:2]))
                if task:
                    self.om.task_started()
                    for step in task.steps:
                        if step.startswith("goto_pose:"):
                            self.pose[:] = [float(v) for v in step.split(":")[1].split(",")]
                    print("robot: executed", task.node_id, task.steps)
                    self.om.task_succeeded()
                state = self.om.build_state(tuple(self.pose), (0.0, 0.0), False, 90.0, False)
            self.client.publish(TOPICS["state"], json.dumps(state))
            time.sleep(0.2)


def main() -> int:
    FakeRobot()
    time.sleep(0.5)
    rc_transport = fleet_manager.main(["transport", "--stations", STATIONS, "--pickup", "pickup_a",
                                       "--drop", "drop_c", "--timeout", "15"])
    rc_endurance = fleet_manager.main(["endurance", "--stations", STATIONS, "--jobs", "5", "--timeout", "15"])
    print(f"RESULT transport rc {rc_transport}, endurance rc {rc_endurance}")
    return rc_transport or rc_endurance


if __name__ == "__main__":
    sys.exit(main())
