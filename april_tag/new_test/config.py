# config.py
import yaml
import numpy as np

# Load calibration
with open("calibration.yaml", "r") as f:
    calib = yaml.safe_load(f)

camera_matrix = np.array(calib["camera_matrix"])
dist_coeffs = np.array(calib["dist_coeffs"])
tag_size = calib["square_size"]  # meters

# Pixhawk connection
connection_string ='/dev/serial0'  #'udp:127.0.0.1:14550'    Use '/dev/serial0'' for simulation
baud_rate = 921600

# AprilTag settings
tag_family = 'tag36h11'

# Landing logic
land_target_timeout = 1.0
min_detections_for_acquire = 3
