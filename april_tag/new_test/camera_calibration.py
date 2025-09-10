# camera_calibration.py
import time
import numpy as np
import cv2
import yaml
from picamera2 import Picamera2

# Chessboard settings
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.025  # 2.5 cm
OUTPUT_FILE = "calibration.yaml"
NUM_IMAGES = 15

def capture_images():
    print("?? Starting Picamera2...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    objpoints = []
    imgpoints = []
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    print(f"?? Take {NUM_IMAGES} photos of the chessboard. Press SPACE when detected. ESC to quit.")

    for i in range(NUM_IMAGES):
        while True:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

            display = frame_bgr.copy()
            if ret:
                cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret)
                status = "? Detected"
            else:
                status = "? Not detected"

            cv2.putText(display, f"Image {i+1}/15 - {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Calibration', display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and ret:
                objpoints.append(objp)
                imgpoints.append(corners)
                print(f"? Captured image {i+1}")
                break
            elif key == 27:
                picam2.stop()
                cv2.destroyAllWindows()
                return

        time.sleep(0.5)

    picam2.stop()
    cv2.destroyAllWindows()

    # Calibrate
    print("?? Calibrating camera...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (640, 480), None, None
    )

    # Save as pure Python lists (no tuples, no numpy)
    data = {
        'camera_matrix': mtx.tolist(),
        'dist_coeffs': dist.flatten().tolist(),  # Ensures 1D list
        'square_size': float(SQUARE_SIZE),
        'image_width': 640,
        'image_height': 480
    }

    with open(OUTPUT_FILE, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)

    print(f"? Calibration saved to {OUTPUT_FILE}")
    print(f"Camera Matrix:\n{mtx}")
    print(f"Distortion Coefficients: {dist.flatten()}")

if __name__ == "__main__":
    capture_images()