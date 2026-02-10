#!/usr/bin/env python3
"""
Both arms teleoperation for real G1 robot
Controls 10 arm motors (left 15-19, right 22-26) from recorded motion
Other 19 motors HOLD their current position
"""
import argparse
import time
import json
import numpy as np
import redis
from robot_control.g1_wrapper import G1RealWorldEnv
from robot_control.config import Config


class BothArmsRealController:
    def __init__(self, config_path, net='eno1', device='cuda'):
        self.redis_client = None
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
            self.redis_pipeline = self.redis_client.pipeline()
        except Exception as e:
            print(f"Error connecting to Redis: {e}")
            exit()
       
        # Use full 29-DOF config
        self.config = Config(config_path)
        self.env = G1RealWorldEnv(net=net, config=self.config)
        
        # Both arms configuration (10 motors total)
        # Left arm: 15-19, Right arm: 22-26
        self.left_arm_indices = [15, 16, 17, 18, 19]
        self.right_arm_indices = [22, 23, 24, 25, 26]
        self.both_arms_indices = self.left_arm_indices + self.right_arm_indices
        
        self.motor_names = {
            15: "left_shoulder_pitch",
            16: "left_shoulder_roll", 
            17: "left_shoulder_yaw",
            18: "left_elbow",
            19: "left_wrist_roll",
            22: "right_shoulder_pitch",
            23: "right_shoulder_roll",
            24: "right_shoulder_yaw",
            25: "right_elbow",
            26: "right_wrist_roll"
        }
        
        self.action_scale = self.config.action_scale
        self.control_dt = self.config.control_dt
        
        # Store initial positions of all joints
        dof_pos, _, _, _, _, _, _ = self.env.get_robot_state()
        self.hold_positions = dof_pos.copy()
        
        print(f"\n{'='*70}")
        print("Both Arms Real Robot Controller")
        print(f"{'='*70}")
        print(f"  Controlled motors: 10 (both arms)")
        print(f"\n  Left arm (5 motors):")
        for idx in self.left_arm_indices:
            print(f"    Motor {idx}: {self.motor_names[idx]}")
        print(f"\n  Right arm (5 motors):")
        for idx in self.right_arm_indices:
            print(f"    Motor {idx}: {self.motor_names[idx]}")
        print(f"\n  Action scale: {self.action_scale}")
        print(f"  Control dt: {self.control_dt} s")
        print(f"  Network interface: {net}")
        print(f"\n  OTHER 19 MOTORS WILL HOLD CURRENT POSITION!")
        print(f"  Initial positions captured.")
        print(f"{'='*70}\n")

    def run(self):
        print(f"{'='*70}")
        print("SAFETY WARNING:")
        print("  - Robot MUST be suspended")
        print("  - Both arms (10 motors) will be controlled")
        print("  - Other 19 joints will HOLD their current position")
        print("  - Press Ctrl+C to stop")
        print("  - Use remote controller [Select] to exit safely")
        print(f"{'='*70}\n")
        
        print("Press [START] on remote to begin...")
        while self.env.read_controller_input().keys != self.env.controller_mapping["start"]:
            time.sleep(0.1)
        
        print("\nStarting both arms motion control...")
        print("Both arms (10 motors): Motion control")
        print("Other joints: Position hold\n")
        
        last_print_time = time.time()
        print_interval = 1.0
        
        # Update hold positions at start
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
                self.redis_pipeline.get("action_body_unitree_g1_with_hands")
                redis_results = self.redis_pipeline.execute()
                
                # Check if data exists
                if redis_results[0] is None:
                    # No motion data, just hold all positions
                    self.env.send_robot_action(self.hold_positions)
                    time.sleep(0.1)
                    continue
                
                # Extract 35-dim mimic_obs
                mimic_obs_35dim = json.loads(redis_results[0])
                
                # Get current robot state
                dof_pos, dof_vel, quat, ang_vel, dof_temp, dof_tau, dof_vol = self.env.get_robot_state()
                
                # Create target positions: start with hold positions
                target_positions = self.hold_positions.copy()
                
                # Extract both arms motors from mimic_obs (joints start at index 6)
                # GMR outputs absolute joint angles, NOT offsets from default
                joints_start_idx = 6
                for motor_idx in self.both_arms_indices:
                    mimic_obs_idx = joints_start_idx + motor_idx
                    action_value = float(mimic_obs_35dim[mimic_obs_idx])
                    # Use action_value directly as absolute angle (no default offset)
                    target_positions[motor_idx] = action_value
                
                # Send command to all motors
                self.env.send_robot_action(target_positions)
                
                # Print status periodically
                current_time = time.time()
                if current_time - last_print_time > print_interval:
                    print(f"[Both Arms Status]")
                    print("  Left Arm:")
                    for motor_idx in self.left_arm_indices:
                        current_angle = float(dof_pos[motor_idx])
                        target_angle = float(target_positions[motor_idx])
                        current_tau = float(dof_tau[motor_idx])
                        print(f"    M{motor_idx:2d} ({self.motor_names[motor_idx]:20s}): "
                              f"Tgt:{target_angle:+.3f} Cur:{current_angle:+.3f} Tau:{current_tau:+.2f}Nm")
                    print("  Right Arm:")
                    for motor_idx in self.right_arm_indices:
                        current_angle = float(dof_pos[motor_idx])
                        target_angle = float(target_positions[motor_idx])
                        current_tau = float(dof_tau[motor_idx])
                        print(f"    M{motor_idx:2d} ({self.motor_names[motor_idx]:20s}): "
                              f"Tgt:{target_angle:+.3f} Cur:{current_angle:+.3f} Tau:{current_tau:+.2f}Nm")
                    print()
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
            
            print("Both arms control finished.")


def main():
    parser = argparse.ArgumentParser(description='Both arms control for real G1 robot')
    parser.add_argument('--config', type=str, 
                        default='robot_control/configs/g1.yaml',
                        help='Path to config file')
    parser.add_argument('--net', type=str, default='enp5s0',
                        help='Network interface for robot communication')
    parser.add_argument('--device', type=str, default='cuda',
                        help='Device (cuda/cpu, not used but kept for compatibility)')
    
    args = parser.parse_args()
    
    print("\n" + "="*70)
    print("G1 Robot - Both Arms Control (10 Motors)")
    print("="*70)
    print(f"Config file: {args.config}")
    print(f"Network interface: {args.net}")
    print()
    print("⚠️  CRITICAL WARNING ⚠️")
    print("  - Robot MUST be suspended!")
    print("  - Both arms (motors 15-19, 22-26) will be controlled")
    print("  - Other 19 motors will HOLD their current position")
    print("="*70)
    print()
    
    confirm = input("Type 'yes' to confirm robot is suspended: ")
    if confirm.lower() != 'yes':
        print("Test cancelled.")
        return
    
    controller = BothArmsRealController(
        config_path=args.config,
        net=args.net,
        device=args.device
    )
    controller.run()


if __name__ == "__main__":
    main()
