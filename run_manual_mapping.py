#!/usr/bin/env python3
"""
Manual mapping runner -- ONE command for hand-driven SLAM.

Launches, in order:
    1. Mine world (Gazebo + TurtleBot3)  -- started UNPAUSED
    2. slam_toolbox
    3. RViz2 (optional)
    4. safety_filter  -- blocks driving into walls (assisted obstacle avoidance)
    5. wasd_teleop    -- arrow-key driving, opened in its OWN terminal window
                         (it needs a focused window to capture key presses)

You drive the robot with the ARROW KEYS in the teleop window to build the map.

In THIS window:
    - press  s  -> save the current map any time
    - press  q  -> quit everything (no save)

Run it:
    python3 run_manual_mapping.py

No need to source anything first -- this sources ROS for every child process
and sets the VM rendering fixes itself.
"""

import os
import sys
import time
import shutil
import signal
import select
import termios
import tty
import threading
import subprocess
from datetime import datetime

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
HOME = os.path.expanduser("~")

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP = os.path.join(HOME, "ros2_ws/install/setup.bash")

RUN_RVIZ = True
MAP_DIR = os.path.join(HOME, "maps")

DELAY_AFTER_WORLD = 12
DELAY_AFTER_SLAM = 8
DELAY_AFTER_RVIZ = 3

LOG_DIR = os.path.join(HOME, ".mine_manual_logs")

CHILD_ENV = os.environ.copy()
CHILD_ENV.update({
    "LIBGL_ALWAYS_SOFTWARE": "1",
    "SVGA_VGPU10": "0",
    "QT_QPA_PLATFORM": "xcb",
    "GAZEBO_MODEL_DATABASE_URI": "",
    "TURTLEBOT3_MODEL": "burger",
})
CHILD_ENV["GAZEBO_MODEL_PATH"] = (
    CHILD_ENV.get("GAZEBO_MODEL_PATH", "")
    + ":/opt/ros/humble/share/turtlebot3_gazebo/models"
    + f":{HOME}/.gazebo/models"
)

STAGES = [
    ("world", "ros2 launch my_bot mine_world.launch.py"),
    ("slam",  "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"),
]
if RUN_RVIZ:
    STAGES.append(("rviz", "rviz2"))

# safety_filter takes teleop's cmd_vel_raw and republishes safe cmd_vel
SAFETY_CMD = "ros2 run my_bot safety_filter"
# teleop publishes to cmd_vel_raw so the safety filter sits in between
TELEOP_CMD = "ros2 run my_bot wasd_teleop --ros-args -r cmd_vel:=cmd_vel_raw"

# ----------------------------------------------------------------------
# Internals
# ----------------------------------------------------------------------
processes = []
stop_event = threading.Event()
save_lock = threading.Lock()
_saving = False


def sourced(cmd):
    return f"source {ROS_SETUP} && source {WS_SETUP} && exec {cmd}"


def launch(name, cmd):
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f"{name}.log")
    log = open(log_path, "w")
    proc = subprocess.Popen(
        ["bash", "-c", sourced(cmd)],
        env=CHILD_ENV, start_new_session=True,
        stdout=log, stderr=subprocess.STDOUT,
    )
    processes.append((name, proc))
    print(f"  [{name}] started (pid {proc.pid}), log: {log_path}")
    return proc


def launch_in_terminal(name, cmd):
    """Open a command in its own terminal window so it can read the keyboard."""
    inner = sourced(cmd).replace('"', '\\"')
    bash_cmd = f'bash -c "{inner}"'

    # Try common terminal emulators in order
    for term in ("gnome-terminal", "xterm", "konsole", "xfce4-terminal"):
        if shutil.which(term):
            if term == "gnome-terminal":
                full = ["gnome-terminal", "--title", name, "--", "bash", "-c", sourced(cmd)]
            elif term == "konsole":
                full = ["konsole", "-e", "bash", "-c", sourced(cmd)]
            elif term == "xfce4-terminal":
                full = ["xfce4-terminal", "--title", name, "-e", bash_cmd]
            else:  # xterm
                full = ["xterm", "-T", name, "-e", "bash", "-c", sourced(cmd)]
            proc = subprocess.Popen(full, env=CHILD_ENV, start_new_session=True)
            processes.append((name, proc))
            print(f"  [{name}] opened in a new {term} window -- click it, then drive with ARROWS")
            return proc

    # No terminal emulator found -> tell the user to run teleop manually
    print("\n  !! No terminal emulator found (gnome-terminal/xterm/konsole).")
    print("     Open a NEW terminal yourself and run:")
    print(f"       source {ROS_SETUP} && source {WS_SETUP}")
    print(f"       {cmd}\n")
    return None


def unpause_gazebo():
    try:
        subprocess.run(["bash", "-c", sourced("gz world -p 0")],
                       env=CHILD_ENV, timeout=10, stderr=subprocess.DEVNULL)
        print("  [gazebo] unpaused (physics running)")
    except Exception:
        pass


def save_map(reason=""):
    global _saving
    with save_lock:
        if _saving:
            return
        _saving = True
    try:
        os.makedirs(MAP_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.join(MAP_DIR, f"mine_map_{stamp}")
        print(f"\n>>> Saving map {('(' + reason + ')') if reason else ''} -> {out}.yaml / .pgm")
        result = subprocess.run(
            ["bash", "-c", sourced(f"ros2 run nav2_map_server map_saver_cli -f {out}")],
            env=CHILD_ENV, timeout=60,
        )
        if result.returncode == 0:
            print(f">>> Map saved: {out}.yaml\n")
        else:
            print(">>> Map save FAILED -- is SLAM still publishing /map?\n")
    except subprocess.TimeoutExpired:
        print(">>> Map save timed out.\n")
    finally:
        with save_lock:
            _saving = False


def keyboard_thread():
    settings = termios.tcgetattr(sys.stdin)
    try:
        while not stop_event.is_set():
            tty.setraw(sys.stdin.fileno())
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            key = sys.stdin.read(1) if r else ""
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            if key == "s":
                save_map("manual")
            elif key == "q":
                print("\n>>> 'q' pressed -- shutting down (no save).")
                stop_event.set()
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def terminate_all():
    print("\n>>> Stopping all processes...")
    for name, proc in reversed(processes):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
    time.sleep(5)
    for name, proc in reversed(processes):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    subprocess.run(["killall", "-9", "gzserver", "gzclient"], stderr=subprocess.DEVNULL)
    print(">>> Done.")


def main():
    print("=" * 60)
    print(" Manual mapping runner")
    print("   Drive with ARROW KEYS in the teleop window.")
    print("   In THIS window:  s = save map   |   q = quit")
    print("=" * 60)

    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    try:
        delays = {"world": DELAY_AFTER_WORLD, "slam": DELAY_AFTER_SLAM,
                  "rviz": DELAY_AFTER_RVIZ}
        for name, cmd in STAGES:
            if stop_event.is_set():
                return
            print(f"\n>>> Launching {name}...")
            launch(name, cmd)
            time.sleep(delays.get(name, 5))
            if name == "world":
                unpause_gazebo()

        if stop_event.is_set():
            return

        # Safety filter (assisted obstacle avoidance) in the background
        print("\n>>> Launching safety_filter (assisted obstacle avoidance)...")
        launch("safety", SAFETY_CMD)
        time.sleep(3)

        # Teleop in its own window so it can capture arrow keys
        print("\n>>> Opening teleop window...")
        launch_in_terminal("teleop", TELEOP_CMD)

        print("\n>>> Ready. Drive with ARROWS in the teleop window.")
        print(">>> Press 's' here to save the map, 'q' here to quit.\n")

        while not stop_event.is_set():
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n>>> Ctrl-C received.")
    finally:
        stop_event.set()
        terminate_all()


if __name__ == "__main__":
    main()
