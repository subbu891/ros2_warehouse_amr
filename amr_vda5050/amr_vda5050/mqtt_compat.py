import paho.mqtt.client as mqtt


def make_client(client_id: str) -> mqtt.Client:
    # paho-mqtt 2.x callback API
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    # paho-mqtt 1.x, shipped with Ubuntu 24.04
    return mqtt.Client(client_id=client_id)
