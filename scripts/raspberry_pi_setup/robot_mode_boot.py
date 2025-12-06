#!/usr/bin/env python3
import subprocess
import time
import datetime
from gpiozero import Button

# --- CONFIGURATION ---
# PIN MAPPING (Based on Toggle Logic)
# Handle Left  -> Activates GPIO 26 -> NAV Mode
PIN_NAV = 26
# Handle Right -> Activates GPIO 21 -> MAP Mode
PIN_MAP = 21 

USER_NAME = "slambot-rpi"
WORKSPACE_DIR = f"/home/{USER_NAME}/slambot_pi_ws"   #Amend as needed

# Launch commands
CMD_NAV = "ros2 launch slambot_bringup real_nav_mode.launch.py"
CMD_MAP = "ros2 launch slambot_bringup real_map_mode.launch.py"
# ---------------------


def _timedatectl_bool(property_name):
    """Return a boolean for the given timedatectl property, or None if unavailable."""
    try:
        value = subprocess.run(
            ["timedatectl", "show", f"--property={property_name}", "--value"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip().lower()
    except subprocess.SubprocessError:
        return None

    if value == "yes":
        return True
    if value == "no":
        return False
    return None


def clock_is_synchronized():
    """Return True when system clock reporting indicates a synced state."""
    system_synced = _timedatectl_bool("SystemClockSynchronized")
    ntp_synced = _timedatectl_bool("NTPSynchronized")

    if system_synced is False or ntp_synced is False:
        return False

    return any(flag is True for flag in (system_synced, ntp_synced))


def wait_for_valid_time():
    """
    Blocks until time is > 2025 AND stable, or times out.
    """
    print("Waiting for System Time Sync (NTP)...", flush=True)

    # 1. Wait for the year to be valid and timedatectl to report sync (max 120 seconds)
    timeout = 120
    start = time.time()

    while time.time() - start < timeout:
        now = datetime.datetime.now()
        if now.year >= 2025 and clock_is_synchronized():
            print("System clock synchronised (timedatectl reports ready).", flush=True)
            print("Waiting 5s for clock slew to settle...", flush=True)
            time.sleep(5)
            print(f"Time Synced! Current Time: {now}", flush=True)
            return
        time.sleep(2)

    print(
        "WARNING: Time sync timed out after "
        f"{timeout}s. Proceeding with unsynced time: {datetime.datetime.now()}",
        flush=True,
    )

def launch_ros_background(command):
    """Starts the ROS launch file in the background."""
    full_command = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE_DIR}/install/setup.bash && "
        f"export ROS_DOMAIN_ID=0 && "
        f"{command}"
    )
    
    print(f"Boot Logic: Starting {command}", flush=True)
    subprocess.Popen(full_command, shell=True, executable='/bin/bash')

def main():
    # 1. PREVENT THE TIME WARP
    wait_for_valid_time()

    # 2. Initialize the pins
    # pull_up=True means the pin floats HIGH.
    # Connecting it to GND (via switch) makes it LOW (is_pressed=True)
    switch_nav = Button(PIN_NAV, pull_up=True)
    switch_map = Button(PIN_MAP, pull_up=True)

    print(f"Reading Mode Switch (Nav=GPIO{PIN_NAV}, Map=GPIO{PIN_MAP})...", flush=True)
    # Increased to 3s to ensure user isn't mid-switch during boot
    time.sleep(3) 

    # 3. DECISION MOMENT
    # Note: If switch is in Middle (OFF), neither will be pressed.
    if switch_nav.is_pressed:
        print(f"Switch detected LEFT (GPIO {PIN_NAV} Active). Mode: NAV", flush=True)
        launch_ros_background(CMD_NAV)
        
    elif switch_map.is_pressed:
        print(f"Switch detected RIGHT (GPIO {PIN_MAP} Active). Mode: MAP", flush=True)
        launch_ros_background(CMD_MAP)
        
    else:
        # Neither is grounded (Middle Position)
        print("Switch detected MIDDLE (OFF). No ROS launch files executed.", flush=True)

    # 4. IDLE MODE
    print("Boot sequence complete. Robot is running. Ignoring further switch changes.", flush=True)
    
    while True:
        time.sleep(60)

if __name__ == "__main__":
    main()