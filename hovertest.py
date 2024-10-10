import dronekit
from dronekit import connect, VehicleMode
import time

# Function to connect to the drone
def connect_drone(connection_string):
    print(f"Connecting to drone on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60)
    
    print("Connection established.")
    print(f" Autopilot Firmware version: {vehicle.version}")
    print(f" Global Location: {vehicle.location.global_frame}")
    print(f" Global Location (relative altitude): {vehicle.location.global_relative_frame}")
    print(f" Local Location: {vehicle.location.local_frame}")
    print(f" Attitude: {vehicle.attitude}")
    print(f" Velocity: {vehicle.velocity}")
    print(f" GPS: {vehicle.gps_0}")
    print(f" Battery: {vehicle.battery}")
    print(f" EKF OK?: {vehicle.ekf_ok}")
    print(f" Last Heartbeat: {vehicle.last_heartbeat}")
    print(f" Is Armable?: {vehicle.is_armable}")
    print(f" System status: {vehicle.system_status.state}")
    print(f" Mode: {vehicle.mode.name}")
    
    # Add a listener for messages
    @vehicle.on_message('STATUSTEXT')
    def listener(self, name, message):
        print(f"Message: {message.text}")
    
    return vehicle

# Function to disable pre-arm checks and GPS
def disable_pre_arm_checks(vehicle):
    print("Disabling pre-arm checks and GPS requirement for testing...")
    try:
        # Disable all pre-arm checks
        vehicle.parameters['ARMING_CHECK'] = 0
        print("Pre-arm checks disabled.")
        
        # Disable GPS
        vehicle.parameters['GPS_TYPE'] = 0
        print("GPS disabled.")
        
        # Wait for parameters to take effect
        time.sleep(2)
    except Exception as e:
        print(f"Error disabling pre-arm checks or GPS: {e}")

# Function to arm the vehicle and set throttle to 50% thrust
def set_throttle_50(vehicle):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    # Set the vehicle mode to an appropriate mode
    print("Setting vehicle mode to ALT_HOLD")
    vehicle.mode = VehicleMode("ALT_HOLD")
    time.sleep(1)  # Allow time for mode change
    
    # Arm the vehicle
    print("Arming motors")
    vehicle.armed = True
    
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Vehicle is armed. Setting throttle to 50%.")
    
    # Set throttle to 50%
    vehicle.channels.overrides['3'] = 1500  # Channel 3 is throttle
    
    print("Throttle set to 50%. Motors should be spinning.")
    time.sleep(5)  # Keep motors spinning for 5 seconds
    
    # Disarm and reset overrides
    vehicle.channels.overrides = {}
    vehicle.armed = False
    print("Motors turned off. Vehicle disarmed.")

# Main execution
if __name__ == '__main__':
    connection_string = 'udp:0.0.0.0:14550'
    vehicle = connect_drone(connection_string)
    
    # Disable pre-arm checks and GPS requirement
    disable_pre_arm_checks(vehicle)
    
    # Try to arm and control the vehicle
    if vehicle.is_armable:
        set_throttle_50(vehicle)
    else:
        print("Vehicle is still not armable after disabling pre-arm checks. Please check manually.")
    
    # Close vehicle connection
    vehicle.close()
    print("Vehicle connection closed.")
