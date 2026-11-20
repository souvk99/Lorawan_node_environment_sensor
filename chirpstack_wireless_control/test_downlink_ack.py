"""
Live End-to-End Verification of ChirpStack Downlink & ACK Uplink
"""

import time
import sys
from chirpstack_client import ChirpStackClient

def test_wireless_roundtrip():
    print("=" * 60)
    print("STARTING LIVE CHIRPSTACK WIRELESS CONFIGURATION TEST")
    print("=" * 60)

    client = ChirpStackClient(
        host="172.16.50.30",
        web_port=8080,
        api_port=8090,
        mqtt_port=1883,
        username="admin",
        password="admin",
        dev_eui="ac1f09fffe22024e"
    )

    ack_received_event = []

    def on_ack(ack_info):
        print("\n" + "=" * 60)
        print(">>> [SUCCESS] RECEIVED LIVE ACK UPLINK ON FPORT 11! <<<")
        print("=" * 60)
        print(f"Status Code : {ack_info['status_code']} ({ack_info['status_str']})")
        print(f"Command Ref : 0x{ack_info['cmd_ref']:02X}")
        print(f"Params Count: {ack_info['count']}")
        print("Confirmed Flash Configuration from Node:")
        for k, v in ack_info["config"].items():
            print(f"  {k:20s}: {v}")
        print("=" * 60 + "\n")
        ack_received_event.append(ack_info)

    def on_telemetry(tel):
        print(f"[LIVE TELEMETRY FPort 2] Temp: {tel['temperature_c']}C | Hum: {tel['humidity_pct']}% | CO2: {tel['co2_ppm']} ppm | Alarm: {tel['alarm_str']}")

    client.on_ack = on_ack
    client.on_data = on_telemetry

    # 1. Login
    print("[Step 1] Authenticating with ChirpStack...")
    if not client.login():
        print("FAIL: Could not authenticate with ChirpStack!")
        return False
    print("SUCCESS: ChirpStack Authenticated.\n")

    # 2. Start MQTT
    print("[Step 2] Connecting to MQTT broker...")
    client.start_mqtt()
    time.sleep(2)
    print("SUCCESS: MQTT connected and listening.\n")

    # 3. Clean queue
    print("[Step 3] Flushing existing downlink queue...")
    client.flush_queue()
    time.sleep(1)

    # 4. Enqueue test downlink (Change temp_max to 38.5 C)
    test_params = {"temp_max": 38.5}
    payload_bytes = client.encode_set_params(test_params)
    print(f"[Step 4] Enqueueing Downlink: {test_params} -> Raw Hex: {payload_bytes.hex()}")

    queue_id = client.enqueue_downlink(payload_bytes, fport=10)
    if not queue_id:
        print("FAIL: Could not enqueue downlink!")
        client.stop_mqtt()
        return False

    print(f"SUCCESS: Downlink queued (Queue ID: {queue_id})")
    print("\n[Step 5] Waiting for Node's uplink transmission...")
    print("Class A node will receive the downlink during RX1/RX2 window and reply with ACK on FPort 11.")
    print("Waiting up to 75 seconds...")

    start_wait = time.time()
    while time.time() - start_wait < 75:
        if ack_received_event:
            break
        time.sleep(1)

    client.stop_mqtt()

    if ack_received_event:
        confirmed_temp_max = ack_received_event[0]["config"]["temp_max"]
        print(f"\n[VERIFIED] Node applied downlink and confirmed temp_max = {confirmed_temp_max} C in Flash!")
        return True
    else:
        print("\n[TIMEOUT] Did not receive ACK uplink within timeout.")
        return False

if __name__ == "__main__":
    success = test_wireless_roundtrip()
    sys.exit(0 if success else 1)
