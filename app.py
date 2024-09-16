import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode

def main():
    # Connect to the vehicle
    vehicle = connect('COM3', baud=57600, wait_ready=True)

    # Initialize PI Controllers
    angle_controller = PIController(kp=0.5, ki=0.1, output_limits=(-10, 10))  # Degrees
    displacement_controller = PIController(kp=0.005, ki=0.001, output_limits=(-0.5, 0.5))  # m/s

    # Arm and take off to a safe altitude
    arm_and_takeoff(3)  # Take off to 3 meters

    # Initialize camera
    cap = cv2.VideoCapture(0)  # Adjust camera index if necessary

    if not cap.isOpened():
        print("Error: Unable to open the camera.")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Unable to capture video frame.")
                break

            current_time = time.time()

            # Process the frame and extract line information
            angle_error, displacement_error = process_frame(frame)

            # Update PI controllers
            angle_output = angle_controller.compute(angle_error, current_time)
            displacement_output = displacement_controller.compute(displacement_error, current_time)

            # Send control commands to the drone
            # Adjust yaw angle
            current_heading = vehicle.heading  # In degrees
            desired_heading = (current_heading + angle_output) % 360
            send_attitude_target(0, 0, desired_heading, thrust=0.5)

            # Adjust lateral movement
            forward_velocity = 0.5  # Constant forward speed
            lateral_velocity = displacement_output  # Left/right movement
            send_velocity(forward_velocity, lateral_velocity, 0)

            # Display the frame with lines (optional)
            cv2.imshow('Line Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.1)  # Control loop delay

    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
