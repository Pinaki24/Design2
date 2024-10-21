import cv2
import numpy as np
import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from PI import PIController
from Line_detection import detect_line_angle_and_displacement
from pymavlink import mavutil

# Create instances of the PI controllers for angle and displacement
angle_pi_controller = PIController(kp=0.5, ki=0.05, output_limits=(-10, 10))
displacement_pi_controller = PIController(kp=0.3, ki=0.03, output_limits=(-0.5, 0.5))

def send_attitude_target(roll_angle, pitch_angle, yaw_angle, thrust):
    # Replace with actual DroneKit commands to set drone attitude
    print(f"send_attitude_target called with roll_angle={roll_angle}, pitch_angle={pitch_angle}, yaw_angle={yaw_angle}, thrust={thrust}")

def change_velocity(velocity_x, velocity_y, velocity_z):
    # Calculate the new target location based on the velocity vectors
    current_location = vehicle.location.global_relative_frame
    north = velocity_x
    east = velocity_y
    down = -velocity_z  # Negative because 'down' needs to be positive in NED
    target_location = LocationGlobalRelative(current_location.lat + north/1e5, current_location.lon + east/1e5, current_location.alt + down)
    vehicle.simple_goto(target_location)

def initiate_landing():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Landing...")
        time.sleep(1)
    print("Landed and disarmed.")

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f}m")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def main():
    # Connect to the drone via WiFi from ESP32
    global vehicle
    vehicle = connect('COM3', baud=57600, wait_ready=True)

    arm_and_takeoff(3)  # Take off to 3 meters

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed and in GUIDED mode.")

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
            line_angle, line_displacement = detect_line_angle_and_displacement()

            # Update PI controllers
            angle_correction = angle_pi_controller.compute(line_angle, current_time)
            displacement_correction = displacement_pi_controller.compute(line_displacement, current_time)

            # Set desired yaw angle and velocities based on corrections
            yaw_angle = angle_correction
            thrust = 0.5  # Maintaining constant thrust
            velocity_x = 0.5  # Moving forward constantly
            velocity_y = displacement_correction
            velocity_z = 0

            # Send commands to drone to update its position and attitude
            send_attitude_target(roll_angle=0, pitch_angle=0, yaw_angle=yaw_angle, thrust=thrust)
            change_velocity(velocity_x, velocity_y, velocity_z)

            # Print outputs for visualization
            print(f"Angle Controller Output: {yaw_angle:.2f} degrees")
            print(f"Displacement Controller Output: {velocity_y:.2f} units")
            print(f"send_attitude_target called with roll_angle=0, pitch_angle=0, yaw_angle={yaw_angle}, thrust={thrust}")
            print(f"send_velocity called with velocity_x={velocity_x}, velocity_y={velocity_y}, velocity_z={velocity_z}")

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

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        vehicle.close()

