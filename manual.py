from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import curses

# Replace 'COM3' with your actual COM port
vehicle = connect('COM3', baud=57600, wait_ready=True)

# Function to send velocity commands to the drone
def send_velocity(velocity_x, velocity_y, velocity_z, yaw_rate):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Ignore position
        0, 0, 0,  # Position
        velocity_x, velocity_y, velocity_z,  # Velocity
        0, 0, 0,  # Acceleration
        yaw_rate, 0  # Yaw rate and yaw anglepip
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to land the drone
def initiate_landing():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Landing...")
        time.sleep(1)
    print("Landed and disarmed.")

# Main function for manual control
def manual_control(screen):
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed and in GUIDED mode.")

    screen.nodelay(True)
    key = ''
    while key != ord('q'):  # Press 'q' to quit
        key = screen.getch()

        velocity_x, velocity_y, yaw_rate = 0, 0, 0
        velocity_z = 0

        if key == curses.KEY_UP:
            velocity_z = -0.5  # Move up
        elif key == curses.KEY_DOWN:
            velocity_z = 0.5  # Move down
        elif key == curses.KEY_LEFT:
            yaw_rate = -0.1  # Turn left
        elif key == curses.KEY_RIGHT:
            yaw_rate = 0.1  # Turn right
        elif key == ord('w'):
            velocity_x = 1  # Move forward
        elif key == ord('s'):
            velocity_x = -1  # Move backward
        elif key == ord('a'):
            velocity_y = -1  # Move left
        elif key == ord('d'):
            velocity_y = 1  # Move right
        elif key == ord('l'):
            initiate_landing()
            break

        send_velocity(velocity_x, velocity_y, velocity_z, yaw_rate)
        time.sleep(0.1)

    vehicle.mode = VehicleMode("LOITER")
    vehicle.close()

# Start manual control using the curses library to capture key presses
curses.wrapper(manual_control)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
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

    # Wait until the vehicle reaches a safe height
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f}m")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
