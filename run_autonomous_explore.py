#!/usr/bin/env python3
"""
Fully autonomous exploration runner -- ONE command does everything.

Launches, in order:
    1. Mine world (Gazebo + TurtleBot3)  -- started UNPAUSED
    2. slam_toolbox
    3. Nav2 navigation (with exploration-tuned params)
    4. RViz2 (optional)
    5. map_kickstart      -- robot spins in place to seed the map (no driving)
    6. frontier_explorer  -- custom autonomous frontier exploration

Then:
    - press  s  -> save the current map any time
    - press  q  -> quit (no save)
    - when exploration finishes on its own -> map auto-saved, stack shuts down

Run it:
    python3 run_autonomous_explore.py

No need to source anything first -- this sources ROS for every child process
and sets the VM rendering fixes itself.
"""

import os
import sys
import time
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
NAV2_PARAMS = os.path.join(HOME, "ros2_ws/src/my_bot/config/nav2_explore.yaml")

RUN_RVIZ = True
MAP_DIR = os.path.join(HOME, "maps")

# Startup delays (seconds). Increase on a slow VM if a stage isn't ready in time.
DELAY_AFTER_WORLD = 12
DELAY_AFTER_SLAM = 15
DELAY_AFTER_NAV2 = 15
DELAY_AFTER_RVIZ = 3
# map_kickstart spins ~14s; wait a little longer before exploring.
DELAY_AFTER_KICKSTART = 17

# The custom frontier_explorer prints this (lower-cased) when done.
COMPLETION_MARKER = "exploration complete"
LOG_DIR = os.path.join(HOME, ".mine_explore_logs")

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
    ("nav2",  f"ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:={NAV2_PARAMS}"),
]
if RUN_RVIZ:
    STAGES.append(("rviz", "rviz2"))

KICKSTART_CMD = "ros2 run my_bot map_kickstart"
EXPLORE_CMD = "ros2 run my_bot frontier_explorer"

# ----------------------------------------------------------------------
# Internals
# ----------------------------------------------------------------------
processes = []
stop_event = threading.Event()
save_lock = threading.Lock()
_saving = False


def sourced(cmd):
    return f"source {ROS_SETUP} && source {WS_SETUP} && exec {cmd}"


def launch(name, cmd, capture=False):
    os.makedirs(LOG_DIR, exist_ok=True)
    full = ["bash", "-c", sourced(cmd)]
    if capture:
        proc = subprocess.Popen(
            full, env=CHILD_ENV, start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
    else:
        log_path = os.path.join(LOG_DIR, f"{name}.log")
        log = open(log_path, "w")
        proc = subprocess.Popen(
            full, env=CHILD_ENV, start_new_session=True,
            stdout=log, stderr=subprocess.STDOUT,
        )
        print(f"  [{name}] started (pid {proc.pid}), log: {log_path}")
    processes.append((name, proc))
    return proc


def unpause_gazebo():
    """Make sure Gazebo physics is running (it sometimes starts paused)."""
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


def explore_monitor_thread(proc):
    for line in proc.stdout:
        sys.stdout.write("[explore] " + line)
        sys.stdout.flush()
        if COMPLETION_MARKER in line.lower():
            print("\n>>> Exploration finished on its own.")
            save_map("auto -- exploration complete")
            stop_event.set()
            break


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
    print(" Fully autonomous exploration runner")
    print("   s = save map   |   q = quit")
    print("=" * 60)

    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    try:
        delays = {"world": DELAY_AFTER_WORLD, "slam": DELAY_AFTER_SLAM,
                  "nav2": DELAY_AFTER_NAV2, "rviz": DELAY_AFTER_RVIZ}
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

        print("\n>>> Kickstart: spinning to seed the map (autonomous)...")
        launch("kickstart", KICKSTART_CMD)
        time.sleep(DELAY_AFTER_KICKSTART)

        if stop_event.is_set():
            return

        print("\n>>> Launching frontier_explorer -- autonomous exploration begins.")
        explore_proc = launch("explore", EXPLORE_CMD, capture=True)
        mon = threading.Thread(target=explore_monitor_thread,
                               args=(explore_proc,), daemon=True)
        mon.start()

        print("\n>>> Running. Press 's' to save, 'q' to quit.\n")

        while not stop_event.is_set():
            if explore_proc.poll() is not None and not stop_event.is_set():
                print("\n>>> frontier_explorer exited. Attempting a final save.")
                save_map("explorer exited")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n>>> Ctrl-C received.")
    finally:
        stop_event.set()
        terminate_all()


if __name__ == "__main__":
    main()
