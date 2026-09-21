import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

def main():
    rclpy.init()
    node = rclpy.create_node("top_camera_acceptance")
    base = "/vision/top_camera"
    states = {}
    subscriptions = []
    for suffix, msg_type in [("rgb", Image),
                             ("depth", Image),
                             ("points", PointCloud2)]:
        topic = f"{base}/{suffix}"
        state = {"count": 0, "stamp": None, "details": "no data", "first": None, "last": None}
        states[topic] = state

        def callback(msg, state=state):
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if not msg.width or not msg.height or not msg.data or stamp == (0, 0):
                return
            if state["stamp"] is not None and stamp <= state["stamp"]:
                return
            state["stamp"] = stamp
            now = time.monotonic()
            if state["first"] is None:
                state["first"] = now
            state["last"] = now
            state["count"] += 1
            encoding = getattr(msg, "encoding", "PointCloud2")
            state["details"] = f"{msg.width}x{msg.height} {encoding}, frame={msg.header.frame_id}"

        subscriptions.append(node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data))
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        passed = True
        now = time.monotonic()
        for topic, state in states.items():
            ok = state["count"] >= 5 and state["last"] is not None and now - state["last"] < 3
            passed = passed and ok
            elapsed = (state["last"] or 0) - (state["first"] or 0)
            hz = (state["count"] - 1) / elapsed if elapsed > 0 else 0
            print(f"{'PASS' if ok else 'FAIL'} {topic}: {state['count']} frames, "
                  f"received ~{hz:.1f} Hz, {state['details']}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())
