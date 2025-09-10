# precision_land.py
import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector
from dronekit import Vehicle
import pymavlink.mavutil as mavutil
from config import *

class PrecisionLander:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.detector = Detector(
            families=tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.last_detection = None
        self.last_detection_time = 0
        self.running = False
        self.thread = None
        self.detection_count = 0

        # For display
        self.display_frame = None
        self.frame_lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.thread.start()
        print("?? PrecisionLander: Started")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        cv2.destroyAllWindows()
        print("?? PrecisionLander: Stopped")

    def _detection_loop(self):
        try:
            print("?? Starting Pi Camera...")
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"size": (640, 480)})
            picam2.configure(config)
            picam2.start()
            time.sleep(2)

            camera_params = (camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2])

            # OpenCV window (for VNC)
            cv2.startWindowThread()
            cv2.namedWindow("AprilTag Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("AprilTag Detection", 640, 480)

            while self.running:
                try:
                    frame = picam2.capture_array()
                    if frame is None:
                        time.sleep(0.1)
                        continue

                    # Convert & undistort
                    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                    if np.any(np.abs(dist_coeffs) > 1e-3):
                        gray = cv2.undistort(gray, camera_matrix, dist_coeffs)
                        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

                    # Detect
                    detections = self.detector.detect(
                        gray,
                        estimate_tag_pose=True,
                        camera_params=camera_params,
                        tag_size=tag_size
                    )

                    now = time.time()
                    tag_seen = False

                    # Work on BGR for display
                    disp_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                    if detections:
                        detection = detections[0]
                        corners = detection.corners.astype(int)

                        # Draw green box
                        cv2.polylines(disp_frame, [corners], True, (0, 255, 0), 2)

                        # Find center of tag
                        center = np.mean(corners, axis=0).astype(int)

                        # Show ID
                        cv2.putText(disp_frame, f"ID {detection.tag_id}",
                                    tuple(corners[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # === Draw alignment arrow ===
                        t = detection.pose_t.flatten()
                        north = -t[1]
                        east = t[0]

                        # Map offset to pixel movement (approx)
                        scale = 100  # 1m ? 100px
                        dx = int(east * scale)  # east ? x
                        dy = int(north * scale)  # north ? y (but camera: forward is up)

                        # Arrow from center to (center - offset) ? points to where drone should go
                        target_x = center[0] - dx
                        target_y = center[1] - dy

                        cv2.arrowedLine(disp_frame, tuple(center), (target_x, target_y),
                                        (255, 0, 0), 3, tipLength=0.3)
                        cv2.putText(disp_frame, "ALIGN", (target_x - 30, target_y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                        # Update detection
                        self.last_detection = (north, east, t[2])
                        self.last_detection_time = now
                        self.detection_count += 1
                        self._send_landing_target(north, east, t[2], now)
                        print(f"?? Tag seen: N={north:+.3f}m, E={east:+.3f}m, D={t[2]:+.3f}m")

                        tag_seen = True
                    else:
                        self.detection_count = max(0, self.detection_count - 1)

                    # Show status
                    status = "DETECTED" if tag_seen else "SEARCHING"
                    color = (0, 255, 0) if tag_seen else (0, 0, 255)
                    cv2.putText(disp_frame, f"Status: {status}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    # Update display
                    with self.frame_lock:
                        self.display_frame = disp_frame.copy()

                    # Show frame
                    cv2.imshow("AprilTag Detection", disp_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False

                    time.sleep(0.05)  # ~20 FPS

                except Exception as e:
                    print(f"??  Detection loop error: {e}")
                    time.sleep(0.1)

        except Exception as e:
            print(f"??  Camera setup error: {e}")
        finally:
            picam2.stop()
            cv2.destroyAllWindows()

    def _send_landing_target(self, north, east, down, timestamp):
        msg = self.vehicle.message_factory.landing_target_encode(
            int(timestamp * 1e6),
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0.0, 0.0,
            down,
            0.0, 0.0,
            north, east, down,
            [1.0, 0.0, 0.0, 0.0],
            2,  # VISION_FIDUCIAL
            1   # position_valid
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def is_target_acquired(self):
        if self.last_detection is None:
            return False
        if time.time() - self.last_detection_time > land_target_timeout:
            return False
        return self.detection_count >= min_detections_for_acquire