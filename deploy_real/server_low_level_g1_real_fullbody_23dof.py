#!/usr/bin/env python3
"""
Full-body teleoperation retargeting for 23-DOF G1 robot.
Controls 23 motors (legs + waist_yaw + arms) from 35-dim mimic_obs via Redis.
Uses the 29-DOF config (SDK preserves motor indices on 23-DOF hardware).
Non-existent motors (13, 14, 20, 21, 27, 28) get hold positions and are ignored by SDK.

Based on server_low_level_g1_real_both_arms.py, extended to include legs and waist.
"""
import argparse
import time
import json
import numpy as np
import redis
from robot_control.g1_wrapper import G1RealWorldEnv
from robot_control.config import Config


# Motor indices that exist on 23-DOF G1
# SDK motor indices are the same as 29-DOF
CONTROLLED_INDICES_23DOF = [
    0, 1, 2, 3, 4, 5,       # left leg (6)
    6, 7, 8, 9, 10, 11,     # right leg (6)
    12,                       # waist yaw (1)
    15, 16, 17, 18, 19,     # left arm (5)
    22, 23, 24, 25, 26,     # right arm (5)
]

# Motor indices that do NOT exist on 23-DOF
MISSING_INDICES_23DOF = [13, 14, 20, 21, 27, 28]

MOTOR_NAMES = {
    0: "left_hip_pitch",
    1: "left_hip_roll",
    2: "left_hip_yaw",
    3: "left_knee",
    4: "left_ankle_pitch",
    5: "left_ankle_roll",
    6: "right_hip_pitch",
    7: "right_hip_roll",
    8: "right_hip_yaw",
    9: "right_knee",
    10: "right_ankle_pitch",
    11: "right_ankle_roll",
    12: "waist_yaw",
    15: "left_shoulder_pitch",
    16: "left_shoulder_roll",
    17: "left_shoulder_yaw",
    18: "left_elbow",
    19: "left_wrist_roll",
    22: "right_shoulder_pitch",
    23: "right_shoulder_roll",
    24: "right_shoulder_yaw",
    25: "right_elbow",
    26: "right_wrist_roll",
}


class FullBody23DOFController:
    def __init__(self, config_path, net='enp5s0', device='cuda'):
        self.redis_client = None
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
            self.redis_pipeline = self.redis_client.pipeline()
        except Exception as e:
            print(f"Error connecting to Redis: {e}")
            exit()

        # Use 29-DOF config (SDK uses same indices on 23-DOF hardware)
        self.config = Config(config_path)
        self.env = G1RealWorldEnv(net=net, config=self.config)

        self.action_scale = self.config.action_scale
        self.control_dt = self.config.control_dt

        # Store initial positions of all joints
        dof_pos, _, _, _, _, _, _ = self.env.get_robot_state()
        self.hold_positions = dof_pos.copy()

        print(f"\n{'='*70}")
        print("Full-Body 23-DOF Real Robot Controller")
        print(f"{'='*70}")
        print(f"  Controlled motors: {len(CONTROLLED_INDICES_23DOF)} (all 23-DOF joints)")
        print(f"\n  Left leg (6 motors):  indices 0-5")
        print(f"  Right leg (6 motors): indices 6-11")
        print(f"  Waist (1 motor):      index 12")
        print(f"  Left arm (5 motors):  indices 15-19")
        print(f"  Right arm (5 motors): indices 22-26")
        print(f"\n  Missing on 23-DOF (ignored): {MISSING_INDICES_23DOF}")
        print(f"\n  Action scale: {self.action_scale}")
        print(f"  Control dt: {self.control_dt} s")
        print(f"  Network interface: {net}")
        print(f"{'='*70}\n")

    def run(self):
        print(f"{'='*70}")
        print("SAFETY WARNING:")
        print("  - Robot MUST be suspended")
        print("  - ALL 23 joints will be controlled by teleoperation")
        print("  - Press Ctrl+C to stop")
        print("  - Use remote controller [Select] to exit safely")
        print(f"{'='*70}\n")

        print("Press [START] on remote to begin...")
        while self.env.read_controller_input().keys != self.env.controller_mapping["start"]:
            time.sleep(0.1)

        print("\nStarting full-body 23-DOF teleoperation...")

        last_print_time = time.time()
        print_interval = 1.0

        # Refresh hold positions at start
        dof_pos, _, _, _, _, _, _ = self.env.get_robot_state()
        self.hold_positions = dof_pos.copy()

        try:
            while True:
                t_start = time.time()

                # Check for exit signal
                if self.env.read_controller_input().keys == self.env.controller_mapping["select"]:
                    print("Select pressed, exiting...")
                    break

                # Get 35-dim mimic_obs from Redis
                # Structure: base_vel_xy(2) + height(1) + roll/pitch(2) + yaw_vel(1) + joints_29(29)
                self.redis_pipeline.get("action_body_unitree_g1_with_hands")
                redis_results = self.redis_pipeline.execute()

                # Check if data exists
                if redis_results[0] is None:
                    # No teleoperation data, hold all positions
                    self.env.send_robot_action(self.hold_positions)
                    time.sleep(0.1)
                    continue

                # Extract 35-dim mimic_obs
                mimic_obs_35dim = json.loads(redis_results[0])

                # Get current robot state
                dof_pos, dof_vel, quat, ang_vel, dof_temp, dof_tau, dof_vol = self.env.get_robot_state()

                # Create target positions: start with hold positions
                # (non-existent motors 13,14,20,21,27,28 keep hold values = ignored by SDK)
                target_positions = self.hold_positions.copy()

                # Map 35-dim mimic_obs to motor commands
                # Joint values in mimic_obs start at index 6
                # mimic_obs[6+i] = absolute joint angle for 29-DOF motor index i
                joints_start_idx = 6
                for motor_idx in CONTROLLED_INDICES_23DOF:
                    mimic_obs_idx = joints_start_idx + motor_idx
                    action_value = float(mimic_obs_35dim[mimic_obs_idx])
                    # Use action_value directly as absolute angle (same as both_arms)
                    target_positions[motor_idx] = action_value

                # Send command to all motors
                self.env.send_robot_action(target_positions)

                # Print status periodically
                current_time = time.time()
                if current_time - last_print_time > print_interval:
                    self._print_status(dof_pos, target_positions, dof_tau)
                    last_print_time = current_time

                # Maintain control frequency
                elapsed = time.time() - t_start
                if elapsed < self.control_dt:
                    time.sleep(self.control_dt - elapsed)

        except KeyboardInterrupt:
            print("\n\nStopping...")
        except Exception as e:
            print(f"\nError in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Hold all positions on exit
            print("Holding all positions...")
            for _ in range(10):
                self.env.send_robot_action(self.hold_positions)
                time.sleep(self.control_dt)

            print("Full-body 23-DOF control finished.")

    def _print_status(self, dof_pos, target_positions, dof_tau):
        """Print periodic status of all controlled joints"""
        print(f"\n[Full-Body 23-DOF Status]")

        groups = [
            ("Left Leg",  [0, 1, 2, 3, 4, 5]),
            ("Right Leg", [6, 7, 8, 9, 10, 11]),
            ("Waist",     [12]),
            ("Left Arm",  [15, 16, 17, 18, 19]),
            ("Right Arm", [22, 23, 24, 25, 26]),
        ]

        for group_name, indices in groups:
            print(f"  {group_name}:")
            for motor_idx in indices:
                name = MOTOR_NAMES.get(motor_idx, f"motor_{motor_idx}")
                cur = float(dof_pos[motor_idx])
                tgt = float(target_positions[motor_idx])
                tau = float(dof_tau[motor_idx])
                print(f"    M{motor_idx:2d} ({name:22s}): "
                      f"Tgt:{tgt:+.3f} Cur:{cur:+.3f} Tau:{tau:+.2f}Nm")
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Full-body 23-DOF teleoperation for real G1 robot')
    parser.add_argument('--config', type=str,
                        default='robot_control/configs/g1.yaml',
                        help='Path to config file (use 29-DOF config)')
    parser.add_argument('--net', type=str, default='enp5s0',
                        help='Network interface for robot communication')
    parser.add_argument('--device', type=str, default='cuda',
                        help='Device (cuda/cpu, not used but kept for compatibility)')

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("G1 Robot - Full-Body 23-DOF Teleoperation")
    print("=" * 70)
    print(f"  Config file: {args.config}")
    print(f"  Network interface: {args.net}")
    print()
    print(f"  Controlled: 23 motors (legs + waist_yaw + both arms)")
    print(f"  Skipped:    6 motors (waist_roll/pitch, wrist_pitch/yaw)")
    print()
    print("  CRITICAL WARNING:")
    print("    - Robot MUST be suspended!")
    print("    - ALL 23 joints will follow teleoperation commands!")
    print("    - Legs WILL move - ensure clearance!")
    print("=" * 70)
    print()

    confirm = input("Type 'yes' to confirm robot is suspended: ")
    if confirm.lower() != 'yes':
        print("Cancelled.")
        return

    controller = FullBody23DOFController(
        config_path=args.config,
        net=args.net,
        device=args.device,
    )
    controller.run()


if __name__ == "__main__":
    main()
