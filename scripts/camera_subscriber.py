import sys
import cv2
import numpy as np
import zmq


def main():
    endpoint = sys.argv[1] if len(sys.argv) > 1 else "tcp://127.0.0.1:5562"
    topic = sys.argv[2] if len(sys.argv) > 2 else "camera.frame"

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, topic)
    sub.setsockopt(zmq.RCVTIMEO, 1000)

    print(f"camera_subscriber: connecting to {endpoint}, topic '{topic}' -- window opens on first frame")
    print("camera_subscriber: press 'q' in the video window, or Ctrl+C here, to stop")

    received = 0
    try:
        while True:
            try:
                _topic, payload = sub.recv_multipart()
            except zmq.Again:
                continue  # just a 1s timeout with nothing received -- normal, keep waiting

            jpeg_bytes = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
            if frame is None:
                continue  # a corrupted/partial frame -- skip it, don't crash

            received += 1
            cv2.imshow("camera_subscriber (press q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\ncamera_subscriber: stopping -- {received} frames received")
        cv2.destroyAllWindows()
        sub.close()
        ctx.term()


if __name__ == "__main__":
    main()