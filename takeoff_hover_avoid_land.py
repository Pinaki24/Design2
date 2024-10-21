"""
This script do the following routine:
    Arm and take off in GUIDED NO GPS mode
    Hold altitude for 30seconds at 1.5m
    While airborne, constantly query distance reading from ultrasonic sensor
    If distance is less than 50cm, decrease the pitch angle to move the drone backward
    Landing is initiated after 30 seconds of testing
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Connect to the Vehicle
print('Connecting to vehicle on udp:0.0.0.0:14550')
vehicle = connect('udp:0.0.0.0:14550', baud=921600,  wait_ready=True)


#### CONSTANTS #####
THRESHOLD_DISTANCE = 0.5 # in meters
DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6
USER_INPUT_ALTITUDE  = 1.5 # in meters
USER_INPUT_FLIGHT_DURATION = 30 #in seconds

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED NO GPS mode
    vehicle.mode    = VehicleMode("GUIDED_NOGPS")
    vehicle.armed   = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")
    thrust = DEFAULT_TAKEOFF_THRUST
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude_and_avoid(thrust = thrust)
        time.sleep(0.2)
        
    time.sleep(1)

# Function to get rangefinder 1 (ultrasonic sensor) to find distance
def get_rangefinder_distance():
    """
    Get the distance reading from Rangefinder 1 (in meters).
    Returns None if no valid reading is available.
    """
    if vehicle.rangefinder:
        distance = vehicle.rangefinder.distance
        print(f"Rangefinder distance: {distance} meters")
        return distance
    else:
        print("Rangefinder data unavailable")
        return None

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_and_avoid_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude_and_avoid(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
            """
    Adjust the pitch angle of the vehicle to avoid obstacles.
    If the rangefinder detects an obstacle closer than the threshold distance, pitch up the vehicle.
    """
    distance = get_rangefinder_distance()
    
    if distance is not None and distance < THRESHOLD_DISTANCE:
        print("Obstacle detected! Adjusting pitch to avoid.")
            
        # Get current pitch angle (in degrees)
        current_pitch = vehicle.attitude.pitch
            
        # Set a pitch adjustment (in radians, positive value to pitch up)
        pitch_adjustment = -0.1 # Adjust as needed (e.g., 0.1 radian = ~5.7 degrees)
            
        # Check if current pitch is greater than the min allowed (to avoid too much negative pitch)
        min_pitch = -0.5  # Example min pitch value in radians (~-28.6 degrees)
        
        # Calculate the new pitch angle
        new_pitch = current_pitch + pitch_adjustment if current_pitch + pitch_adjustment >= min_pitch else min_pitch
            
        # Set the new pitch by replacing the argument of set_attitude function
        pitch_angle = new_pitch
        print(f"New pitch angle set to: {new_pitch} radians")
    else:
        print("No obstacle detected, no pitch adjustment needed.")
        pitch_angle = current_pitch
        
    send_attitude_target(roll_angle, pitch_angle,
                            yaw_angle, yaw_rate, False,
                            thrust)
    time.sleep(0.1)
    
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)
    

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

            
# Take off 1.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(USER_INPUT_ALTITUDE)

# Hold the position for 30 seconds and do obstacle avoidance if required.
print("Hold position for 30 seconds")
set_attitude_and_avoid(duration = USER_INPUT_FLIGHT_DURATION)


print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
