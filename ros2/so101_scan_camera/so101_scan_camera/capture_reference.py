"""capture_reference: take a dark or a white with line_camera.

    ros2 run so101_scan_camera capture_reference white       # PTFE under the scan's lamp
    ros2 run so101_scan_camera capture_reference dark        # lens capped, same exposure

Hold the mirror still and point the head at the reference. Without --directory the frames go
into the recording that is open, or else into line_camera's reference_dir
($SO101_SCAN_DATA/references), which the next scan then names in its scan.json, so
scan_to_dataset finds them. Take the white first: if it says the peak is under half of full
scale or saturates, change exposure_us and take it again, then take the dark at that exposure.
"""

import argparse
import sys
import threading

import rclpy

from so101_scan_interfaces.srv import CaptureReference


def main(argv=None):
    ap = argparse.ArgumentParser(description="Average a dark or a white with line_camera")
    ap.add_argument("kind", choices=("dark", "white"))
    ap.add_argument("--name", default="", help="folder name under reference/ (default: the kind)")
    ap.add_argument("--frames", type=int, default=16, help="frames to average")
    ap.add_argument("--directory", default="", help="scan or reference folder (default: see above)")
    ap.add_argument("--node", default="/line_camera")
    args, _ = ap.parse_known_args(argv)
    rclpy.init()
    node = rclpy.create_node("capture_reference")
    try:
        client = node.create_client(CaptureReference, args.node + "/capture_reference")
        if not client.wait_for_service(timeout_sec=5.0):
            print("%s/capture_reference isn't there: is line_camera running?" % args.node, file=sys.stderr)
            return 1
        req = CaptureReference.Request(directory=args.directory, kind=args.kind, name=args.name, frames=args.frames)
        future = client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        while not done.is_set():
            executor.spin_once(timeout_sec=0.1)
        res = future.result()
        print(res.message, file=sys.stdout if res.success else sys.stderr)
        return 0 if res.success else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
