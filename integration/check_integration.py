
import argparse
import csv
import time

from pymavlink import mavutil


def request_message(master, message_id, frequency):  
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
        0,
        message_id,
        1_000_000 / frequency,
        0, 0, 0, 0, 0
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conn", default="udpin:127.0.0.1:14552")
    parser.add_argument("--output", default="integration_log.csv")
    args = parser.parse_args()

    print("Connecting to ArduPilot...")

    master = mavutil.mavlink_connection(args.conn)
    master.wait_heartbeat()

    print("Connected to ArduPilot!")

    request_message(master, 193, 5)  # EKF_STATUS_REPORT
    request_message(master, 32, 5)   # LOCAL_POSITION_NED

    start_time = time.time()
    last_mode = None

    records = []
    position = {"x": 0, "y": 0, "z": 0}

    try:
        while True:
            msg = master.recv_match(
                type=[
                    "HEARTBEAT",
                    "EKF_STATUS_REPORT",
                    "LOCAL_POSITION_NED",
                    "STATUSTEXT"
                ],
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            msg_type = msg.get_type()
            current_time = round(time.time() - start_time, 2)

            if msg_type == "HEARTBEAT":
                if msg.get_srcComponent() != 1:
                    continue
                mode = mavutil.mode_string_v10(msg)

                if mode != last_mode:
                    print(f"[{current_time}s] Mode: {mode}")
                    last_mode = mode

            elif msg_type == "LOCAL_POSITION_NED":
                position["x"] = msg.x
                position["y"] = msg.y
                position["z"] = msg.z

            elif msg_type == "EKF_STATUS_REPORT":
                print(
                    f"[{current_time}s] "
                    f"EKF position variance: "
                    f"{msg.pos_horiz_variance:.4f}"
                )

                records.append({
                    "time": current_time,
                    "mode": last_mode,
                    "position_variance": msg.pos_horiz_variance,
                    "velocity_variance": msg.velocity_variance,
                    "x": position["x"],
                    "y": position["y"],
                    "z": position["z"]
                })

            elif msg_type == "STATUSTEXT":
                message = msg.text
                print(f"[{current_time}s] STATUS: {message}")

    except KeyboardInterrupt:
        print("\nStopping integration monitor...")

    if records:
        with open(args.output, "w", newline="") as file:
            writer = csv.DictWriter(
                file,
                fieldnames=records[0].keys()
            )
            writer.writeheader()
            writer.writerows(records)

        print(f"Saved {len(records)} records to {args.output}")
    else:
        print("No EKF data received.")


if __name__ == "__main__":
    main()
