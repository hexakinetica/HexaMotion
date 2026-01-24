#!/usr/bin/env python3
import socket
import time
import threading
import re
import argparse
from typing import Dict, List

# --- Configuration ---
LISTEN_IP = "127.0.0.1"
COMMAND_PORT = 50001
FEEDBACK_PORT = 50002
DEBUG_PORT = 60000

NUM_AXES = 6
FEEDBACK_RATE_HZ = 250.0  # High frequency to mimic a real controller

# --- Global State ---
class RobotState:
    def __init__(self):
        self.joint_angles_deg: Dict[int, float] = {i: 0.0 for i in range(1, NUM_AXES + 1)}
        self.digital_inputs: List[bool] = [False] * 8
        self.estop_active: bool = False
        self.power_on: bool = True
        self.lock = threading.Lock()

robot_state = RobotState()

# ==============================================================================
# --- EMULATOR LOGIC ---
# ==============================================================================

def parse_command(data: bytes) -> Dict[int, float]:
    """Parses a command string like '[CMD] A1=10.5 A2=-20.0' into a dictionary."""
    command_dict = {}
    try:
        command_str = data.decode('utf-8').strip()
        if not command_str.startswith('[CMD]'):
            return {}
            
        # Find all key=value pairs
        pairs = re.findall(r"A(\d+)=(-?[\d.]+)", command_str)
        for axis_str, angle_str in pairs:
            axis = int(axis_str)
            angle = float(angle_str)
            if 1 <= axis <= NUM_AXES:
                command_dict[axis] = angle
    except (UnicodeDecodeError, ValueError) as e:
        print(f"[Emulator:Parser] Error parsing command: {e}")
    return command_dict

def command_listener_thread():
    """Listens for incoming UDP command packets and updates the robot state."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((LISTEN_IP, COMMAND_PORT))
            print(f"[Emulator] Listening for commands on {LISTEN_IP}:{COMMAND_PORT}")

            while True:
                data, addr = sock.recvfrom(1024)
                print(f"[Emulator] RX Command: {data.decode().strip()}")
                
                commands = parse_command(data)
                if commands:
                    with robot_state.lock:
                        for axis, angle in commands.items():
                            robot_state.joint_angles_deg[axis] = angle
        except OSError as e:
            print(f"[Emulator] ERROR: Could not bind to port {COMMAND_PORT}. Is another instance running? Details: {e}")
            return

def feedback_sender_thread():
    """Periodically sends the current robot state via UDP."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        print(f"[Emulator] Sending feedback to {LISTEN_IP}:{FEEDBACK_PORT} at {FEEDBACK_RATE_HZ} Hz")
        
        period = 1.0 / FEEDBACK_RATE_HZ
        while True:
            start_time = time.monotonic()
            try:
                with robot_state.lock:
                    # Format: "[FB] A1=10.0,A2=20.0,DI1=0,DI2=1,ESTOP=0,POWER=1"
                    parts = ["[FB]"]
                    for i in range(1, NUM_AXES + 1):
                        parts.append(f"A{i}={robot_state.joint_angles_deg[i]:.4f}")
                    
                    for i, di_state in enumerate(robot_state.digital_inputs, 1):
                        parts.append(f"DI{i}={1 if di_state else 0}")
                        
                    parts.append(f"ESTOP={1 if robot_state.estop_active else 0}")
                    parts.append(f"POWER={1 if robot_state.power_on else 0}")
                    
                feedback_str = ",".join(parts)
                sock.sendto(feedback_str.encode('utf-8'), (LISTEN_IP, FEEDBACK_PORT))
            except Exception as e:
                print(f"[Emulator] Feedback sender error: {e}")
            
            elapsed = time.monotonic() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

# ==============================================================================
# --- DEBUG LISTENER LOGIC ---
# ==============================================================================

def debug_listener_thread():
    """Listens for and prints any incoming UDP debug packets."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((LISTEN_IP, DEBUG_PORT))
            print(f"[Debug] Listening for debug packets on {LISTEN_IP}:{DEBUG_PORT}")

            while True:
                data, addr = sock.recvfrom(2048)
                print(f"[Debug] RX from {addr}: {data.decode('utf-8').strip()}")
        except OSError as e:
            print(f"[Debug] ERROR: Could not bind to port {DEBUG_PORT}. Is another instance running? Details: {e}")
            return

# ==============================================================================
# --- MAIN EXECUTION ---
# ==============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Universal Robot Hardware Emulator and Debug Utility.")
    parser.add_argument('--emulator', action='store_true', help='Run the robot hardware emulator.')
    parser.add_argument('--debug', action='store_true', help='Run the debug packet listener.')
    args = parser.parse_args()

    threads = []
    if args.emulator:
        print("--- Starting Robot Hardware Emulator ---")
        threads.append(threading.Thread(target=command_listener_thread, daemon=True))
        threads.append(threading.Thread(target=feedback_sender_thread, daemon=True))
    
    if args.debug:
        print("--- Starting Debug Packet Listener ---")
        threads.append(threading.Thread(target=debug_listener_thread, daemon=True))

    if not threads:
        print("No mode selected. Use --emulator or --debug. Use --emulator --debug to run both.")
        parser.print_help()
        exit(1)

    for t in threads:
        t.start()

    print("\nUtility running. Press Ctrl+C to exit.")
    try:
        while all(t.is_alive() for t in threads):
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down.")