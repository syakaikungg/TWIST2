#!/usr/bin/env python3
"""
TWIST2 23DOF G1 Real Robot Controller with RL Policy

23DOF policy-based controller for real G1 robot.
Uses the trained 23DOF ONNX policy for stable walking/motion control.

Based on server_low_level_g1_real.py (29DOF) adapted for 23DOF policy.
Uses 29DOF config (g1.yaml) for robot interface, with 23→29 motor mapping.

23DOF joint order (policy output):
  [0-5]:   left leg  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
  [6-11]:  right leg
  [12]:    waist_yaw
  [13-17]: left arm  (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
  [18-22]: right arm

Usage:
    python server_low_level_g1_real_23dof.py --policy /path/to/23dof.onnx --net enp5s0
"""
import argparse
import time
import json
import os
import numpy as np
import torch
import redis
from collections import deque
from rich import print

from robot_control.g1_wrapper import G1RealWorldEnv
from robot_control.config import Config
from data_utils.rot_utils import quatToEuler

try:
    import onnxruntime as ort
except ImportError:
    ort = None


# ---- 23DOF ↔ 29DOF motor mapping ----
# 23DOF joint index → 29DOF motor index
_23TO29_MOTOR_MAP = [
    0, 1, 2, 3, 4, 5,       # left leg (6)
    6, 7, 8, 9, 10, 11,     # right leg (6)
    12,                       # waist_yaw (1)
    15, 16, 17, 18, 19,     # left arm (5) — skip motors 13,14
    22, 23, 24, 25, 26,     # right arm (5) — skip motors 20,21,27,28
]

# 29DOF index → 23DOF index (None if not in 23DOF)
_29TO23_JOINT_MAP = [None] * 29
for _j23, _m29 in enumerate(_23TO29_MOTOR_MAP):
    _29TO23_JOINT_MAP[_m29] = _j23

# Motors that do NOT exist on 23DOF hardware
_MISSING_MOTORS = [13, 14, 20, 21, 27, 28]


class OnnxPolicyWrapper:
    """Minimal wrapper so ONNXRuntime policies mimic TorchScript call signature."""

    def __init__(self, session, input_name, output_index=0):
        self.session = session
        self.input_name = input_name
        self.output_index = output_index

    def __call__(self, obs_tensor: torch.Tensor) -> torch.Tensor:
        if isinstance(obs_tensor, torch.Tensor):
            obs_np = obs_tensor.detach().cpu().numpy()
        else:
            obs_np = np.asarray(obs_tensor, dtype=np.float32)
        outputs = self.session.run(None, {self.input_name: obs_np})
        result = outputs[self.output_index]
        if not isinstance(result, np.ndarray):
            result = np.asarray(result, dtype=np.float32)
        return torch.from_numpy(result.astype(np.float32))


class EMASmoother:
    """Exponential Moving Average smoother for body actions."""

    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.initialized = False
        self.smoothed_value = None

    def smooth(self, new_value):
        if not self.initialized:
            self.smoothed_value = new_value.copy()
            self.initialized = True
            return self.smoothed_value
        self.smoothed_value = self.alpha * new_value + (1 - self.alpha) * self.smoothed_value
        return self.smoothed_value

    def reset(self):
        self.initialized = False
        self.smoothed_value = None


def load_onnx_policy(policy_path: str, device: str) -> OnnxPolicyWrapper:
    if ort is None:
        raise ImportError("onnxruntime is required but not installed.")
    providers = []
    available = ort.get_available_providers()
    if device.startswith('cuda') and 'CUDAExecutionProvider' in available:
        providers.append('CUDAExecutionProvider')
    providers.append('CPUExecutionProvider')
    session = ort.InferenceSession(policy_path, providers=providers)
    input_name = session.get_inputs()[0].name
    print(f"ONNX policy loaded from {policy_path}")
    print(f"  Providers: {session.get_providers()}")
    print(f"  Input:  {session.get_inputs()[0].shape}")
    print(f"  Output: {session.get_outputs()[0].shape}")
    return OnnxPolicyWrapper(session, input_name)


class RealTimePolicyController23DOF:
    """
    23DOF real robot controller with RL walking policy.
    Uses 29DOF config for robot communication, maps to/from 23DOF policy.
    """

    def __init__(self,
                 policy_path,
                 config_path,
                 device='cuda',
                 net='enp5s0',
                 record_proprio=False,
                 smooth_body=0.0):

        # ---- Redis ----
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.redis_pipeline = self.redis_client.pipeline()

        # ---- Robot (29DOF interface) ----
        self.config = Config(config_path)
        self.env = G1RealWorldEnv(net=net, config=self.config)

        # ---- Policy (23DOF) ----
        self.device = device
        self.policy = load_onnx_policy(policy_path, device)
        self.num_actions = 23

        # 23DOF default joint positions
        self.default_dof_pos_23 = np.array([
            -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,   # left leg (6)
            -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,   # right leg (6)
            0.0,                                 # waist_yaw (1)
            0.0, 0.4, 0.0, 1.2, 0.0,           # left arm (5)
            0.0, -0.4, 0.0, 1.2, 0.0,          # right arm (5)
        ], dtype=np.float32)

        # 29DOF default (for motors that the policy doesn't control)
        self.default_dof_pos_29 = self.config.default_angles.copy()

        # Scaling
        self.ang_vel_scale = 0.25
        self.dof_vel_scale = 0.05
        self.dof_pos_scale = 1.0
        self.action_scale = np.array([0.5] * 23, dtype=np.float32)
        self.ankle_idx = [4, 5, 10, 11]   # in 23DOF indexing

        # ---- Observation structure (must match trained 23DOF model) ----
        self.n_mimic_obs = 6 + 23       # 29
        self.n_proprio = 3 + 2 + 3 * 23  # 74
        self.n_obs_single = self.n_mimic_obs + self.n_proprio  # 103
        self.history_len = 10
        self.total_obs_size = self.n_obs_single * (self.history_len + 1) + self.n_mimic_obs  # 1162

        print(f"\nTWIST2 23DOF Real Controller:")
        print(f"  num_actions:    {self.num_actions}")
        print(f"  n_mimic_obs:    {self.n_mimic_obs}")
        print(f"  n_proprio:      {self.n_proprio}")
        print(f"  n_obs_single:   {self.n_obs_single}")
        print(f"  history_len:    {self.history_len}")
        print(f"  total_obs_size: {self.total_obs_size}")

        # History buffer
        self.proprio_history_buf = deque(maxlen=self.history_len)
        for _ in range(self.history_len):
            self.proprio_history_buf.append(np.zeros(self.n_obs_single, dtype=np.float32))

        self.last_action = np.zeros(self.num_actions, dtype=np.float32)

        # Control timing
        self.control_dt = self.config.control_dt

        # Recording
        self.record_proprio = record_proprio
        self.proprio_recordings = [] if record_proprio else None

        # Smoothing
        self.body_smoother = EMASmoother(alpha=smooth_body) if smooth_body > 0.0 else None
        if self.body_smoother:
            print(f"  Body smoothing: alpha={smooth_body}")

    # ---- helpers ----

    def _extract_23dof_from_29(self, dof_29):
        """Extract 23DOF joint values from 29DOF motor array."""
        return np.array([dof_29[m] for m in _23TO29_MOTOR_MAP], dtype=np.float32)

    def _expand_23_to_29(self, action_23, hold_29):
        """Map 23DOF target to 29DOF motor array. Missing motors keep hold values."""
        target_29 = hold_29.copy()
        for j23, m29 in enumerate(_23TO29_MOTOR_MAP):
            target_29[m29] = action_23[j23]
        return target_29

    # ---- main ----

    def reset_robot(self):
        print("Press [START] on remote to move to default position ...")
        self.env.move_to_default_pos()
        print("In default position. Press [A] to start policy ...")
        self.env.default_pos_state()

    def run(self):
        self.reset_robot()
        print("\n[bold green]23DOF policy loop started. Press [Select] to exit.[/bold green]\n")

        try:
            while True:
                t_start = time.time()

                # ---- exit check ----
                if self.env.read_controller_input().keys == self.env.controller_mapping["select"]:
                    print("[yellow]Select pressed — exiting.[/yellow]")
                    break

                # ---- read robot state (29DOF) ----
                dof_pos_29, dof_vel_29, quat, ang_vel, dof_temp, dof_tau, dof_vol = \
                    self.env.get_robot_state()

                # Convert to 23DOF
                dof_pos = self._extract_23dof_from_29(dof_pos_29)
                dof_vel = self._extract_23dof_from_29(dof_vel_29)

                rpy = quatToEuler(quat)

                # ---- build proprio obs (23DOF) ----
                obs_dof_vel = dof_vel.copy()
                obs_dof_vel[self.ankle_idx] = 0.0

                obs_proprio = np.concatenate([
                    ang_vel * self.ang_vel_scale,           # 3
                    rpy[:2],                                # 2  (roll, pitch)
                    (dof_pos - self.default_dof_pos_23),    # 23
                    obs_dof_vel * self.dof_vel_scale,       # 23
                    self.last_action,                       # 23
                ])  # total: 74

                # ---- send state to Redis ----
                state_body = np.concatenate([
                    ang_vel,        # 3
                    rpy[:2],        # 2
                    dof_pos,        # 23
                ])  # total: 28
                self.redis_pipeline.set("state_body_unitree_g1_23dof",
                                        json.dumps(state_body.tolist()))
                self.redis_pipeline.set("t_state", int(time.time() * 1000))
                self.redis_pipeline.execute()

                # ---- get mimic obs from Redis (29 dims) ----
                self.redis_pipeline.get("action_body_unitree_g1_23dof")
                redis_results = self.redis_pipeline.execute()

                if redis_results[0] is not None:
                    action_mimic = np.array(json.loads(redis_results[0]),
                                            dtype=np.float32)
                else:
                    action_mimic = np.zeros(self.n_mimic_obs, dtype=np.float32)

                # Optional smoothing
                if self.body_smoother is not None:
                    action_mimic = self.body_smoother.smooth(action_mimic)

                # ---- assemble observation buffer ----
                obs_full = np.concatenate([action_mimic, obs_proprio])   # 103
                obs_hist = np.array(self.proprio_history_buf).flatten()  # 1030
                self.proprio_history_buf.append(obs_full)
                future_obs = action_mimic.copy()                        # 29

                obs_buf = np.concatenate([obs_full, obs_hist, future_obs])  # 1162
                assert obs_buf.shape[0] == self.total_obs_size, \
                    f"Expected {self.total_obs_size} obs, got {obs_buf.shape[0]}"

                # ---- run policy ----
                obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to(self.device)
                with torch.no_grad():
                    raw_action = self.policy(obs_tensor).cpu().numpy().squeeze()

                self.last_action = raw_action.copy()
                raw_action = np.clip(raw_action, -10.0, 10.0)

                # ---- compute target joint positions (23DOF) ----
                target_dof_pos_23 = self.default_dof_pos_23 + raw_action * self.action_scale

                # ---- map to 29 motors and send ----
                target_dof_pos_29 = self._expand_23_to_29(target_dof_pos_23,
                                                          self.default_dof_pos_29)
                self.env.send_robot_action(target_dof_pos_29)

                # ---- recording ----
                if self.record_proprio:
                    self.proprio_recordings.append({
                        'timestamp': time.time(),
                        'dof_pos_23': dof_pos.tolist(),
                        'target_23': target_dof_pos_23.tolist(),
                        'raw_action': raw_action.tolist(),
                        'temperature': dof_temp.tolist(),
                        'tau': dof_tau.tolist(),
                        'voltage': dof_vol.tolist(),
                    })

                # ---- timing ----
                elapsed = time.time() - t_start
                if elapsed < self.control_dt:
                    time.sleep(self.control_dt - elapsed)

        except KeyboardInterrupt:
            print("\n[yellow]Ctrl+C — stopping.[/yellow]")
        except Exception as e:
            print(f"[red]Error: {e}[/red]")
            import traceback
            traceback.print_exc()
        finally:
            if self.record_proprio and self.proprio_recordings:
                os.makedirs('logs', exist_ok=True)
                ts = time.strftime("%Y%m%d_%H%M%S")
                fname = f'logs/twist2_real_23dof_{ts}.json'
                with open(fname, 'w') as f:
                    json.dump(self.proprio_recordings, f)
                print(f"Recordings saved: {fname}")
            self.env.close()
            print("23DOF real controller finished.")


def main():
    parser = argparse.ArgumentParser(
        description='TWIST2 23DOF RL policy on real G1 robot')
    parser.add_argument('--policy', type=str, required=True,
                        help='Path to 23DOF ONNX policy')
    parser.add_argument('--config', type=str,
                        default='robot_control/configs/g1.yaml',
                        help='Robot config (use 29DOF config)')
    parser.add_argument('--device', type=str, default='cuda')
    parser.add_argument('--net', type=str, default='enp5s0',
                        help='Network interface to robot')
    parser.add_argument('--record_proprio', action='store_true')
    parser.add_argument('--smooth_body', type=float, default=0.0,
                        help='EMA smoothing alpha (0=off, 0.5=moderate)')

    args = parser.parse_args()

    if not os.path.exists(args.policy):
        print(f"[red]Error: policy not found: {args.policy}[/red]")
        return

    print("=" * 60)
    print("TWIST2 23DOF Real Robot Controller (RL Policy)")
    print("=" * 60)
    print(f"  Policy:    {args.policy}")
    print(f"  Config:    {args.config}")
    print(f"  Device:    {args.device}")
    print(f"  Network:   {args.net}")
    print(f"  Smoothing: {args.smooth_body}")
    print()
    print("  Controlled: 23 motors via RL policy")
    print(f"  Skipped:    {_MISSING_MOTORS} (hold at default)")
    print("=" * 60)

    controller = RealTimePolicyController23DOF(
        policy_path=args.policy,
        config_path=args.config,
        device=args.device,
        net=args.net,
        record_proprio=args.record_proprio,
        smooth_body=args.smooth_body,
    )
    controller.run()


if __name__ == "__main__":
    main()
