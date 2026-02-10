"""
TWIST2 23DOF G1 Simulation Controller

23DOF関節構成:
[0-5]:   左脚 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
[6-11]:  右脚
[12]:    waist_yaw
[13-17]: 左腕 (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
[18-22]: 右腕

使用方法:
    python server_low_level_g1_23dof_sim.py --policy /path/to/23dof_policy.onnx
"""

import argparse
import json
import time
import numpy as np
import redis
import mujoco
import torch
from rich import print
from collections import deque
import mujoco.viewer as mjv
from tqdm import tqdm
import os
from data_utils.rot_utils import quatToEuler

try:
    import onnxruntime as ort
except ImportError:
    ort = None


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


def load_onnx_policy(policy_path: str, device: str) -> OnnxPolicyWrapper:
    if ort is None:
        raise ImportError("onnxruntime is required for ONNX policy inference but is not installed.")
    providers = []
    available = ort.get_available_providers()
    if device.startswith('cuda'):
        if 'CUDAExecutionProvider' in available:
            providers.append('CUDAExecutionProvider')
        else:
            print("CUDAExecutionProvider not available in onnxruntime; falling back to CPUExecutionProvider.")
    providers.append('CPUExecutionProvider')
    session = ort.InferenceSession(policy_path, providers=providers)
    input_name = session.get_inputs()[0].name
    print(f"ONNX policy loaded from {policy_path} using providers: {session.get_providers()}")
    return OnnxPolicyWrapper(session, input_name)


class RealTimePolicyController23DOF:
    """23DOF G1用のリアルタイムポリシーコントローラ"""
    
    def __init__(self, 
                 xml_file, 
                 policy_path, 
                 device='cuda', 
                 record_video=False,
                 record_proprio=False,
                 measure_fps=False,
                 limit_fps=True,
                 policy_frequency=50,
                 ):
        self.measure_fps = measure_fps
        self.limit_fps = limit_fps
        self.redis_client = None
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
            self.redis_pipeline = self.redis_client.pipeline()
        except Exception as e:
            print(f"Error connecting to Redis: {e}")

        self.device = device
        self.policy = load_onnx_policy(policy_path, device)

        # Create MuJoCo sim
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.model.opt.timestep = 0.001
        self.data = mujoco.MjData(self.model)
        
        self.viewer = mjv.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = 0
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 0
        self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = 0
        self.viewer.cam.distance = 2.0

        # 23DOF Configuration
        self.num_actions = 23
        self.sim_duration = 100000.0
        self.sim_dt = 0.001
        self.sim_decimation = 1 / (policy_frequency * self.sim_dt)
        print(f"sim_decimation: {self.sim_decimation}")

        self.last_action = np.zeros(self.num_actions, dtype=np.float32)

        # 23DOF default positions
        self.default_dof_pos = np.array([
            -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)
            -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)
            0.0,                              # waist_yaw (1)
            0.0, 0.4, 0.0, 1.2, 0.0,         # left arm (5)
            0.0, -0.4, 0.0, 1.2, 0.0,        # right arm (5)
        ])

        # MuJoCo initial pose (root 7 + joints 23)
        self.mujoco_default_dof_pos = np.concatenate([
            np.array([0, 0, 0.793]),          # root position
            np.array([1, 0, 0, 0]),           # root quaternion
            np.array([
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)
                0.0,                              # waist_yaw (1)
                0.0, 0.2, 0.0, 1.2, 0.0,         # left arm (5)
                0.0, -0.2, 0.0, 1.2, 0.0,        # right arm (5)
            ])
        ])

        # PD gains (23DOF)
        self.stiffness = np.array([
            100, 100, 100, 150, 40, 40,  # left leg
            100, 100, 100, 150, 40, 40,  # right leg
            150,                          # waist_yaw
            40, 40, 40, 40, 4.0,         # left arm
            40, 40, 40, 40, 4.0,         # right arm
        ])
        self.damping = np.array([
            2, 2, 2, 4, 2, 2,  # left leg
            2, 2, 2, 4, 2, 2,  # right leg
            4,                  # waist_yaw
            5, 5, 5, 5, 0.2,   # left arm
            5, 5, 5, 5, 0.2,   # right arm
        ])
        
        self.torque_limits = np.array([
            100, 100, 100, 150, 40, 40,  # left leg
            100, 100, 100, 150, 40, 40,  # right leg
            150,                          # waist_yaw
            40, 40, 40, 40, 4.0,         # left arm
            40, 40, 40, 40, 4.0,         # right arm
        ])

        self.action_scale = np.array([0.5] * 23)
        self.ankle_idx = [4, 5, 10, 11]

        # Observation dimensions (23DOF)
        self.n_mimic_obs = 6 + 23  # 29 (root_vel_xy + root_pos_z + roll_pitch + yaw_ang_vel + dof_pos)
        self.n_proprio = 3 + 2 + 3 * 23  # 74 (ang_vel + imu + dof_pos + dof_vel + action)
        self.n_obs_single = self.n_mimic_obs + self.n_proprio  # 103
        self.history_len = 10
        
        self.total_obs_size = self.n_obs_single * (self.history_len + 1) + self.n_mimic_obs  # 103*11 + 29 = 1162

        print(f"TWIST2 23DOF Controller Configuration:")
        print(f"  num_actions: {self.num_actions}")
        print(f"  n_mimic_obs: {self.n_mimic_obs}")
        print(f"  n_proprio: {self.n_proprio}")
        print(f"  n_obs_single: {self.n_obs_single}")
        print(f"  history_len: {self.history_len}")
        print(f"  total_obs_size: {self.total_obs_size}")

        # Initialize history buffer
        self.proprio_history_buf = deque(maxlen=self.history_len)
        for _ in range(self.history_len):
            self.proprio_history_buf.append(np.zeros(self.n_obs_single, dtype=np.float32))

        # Recording
        self.record_video = record_video
        self.record_proprio = record_proprio
        self.proprio_recordings = [] if record_proprio else None
        

    def reset_sim(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def reset(self, init_pos):
        """Reset robot to initial position"""
        self.data.qpos[:] = init_pos
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def extract_data(self):
        """Extract robot state data"""
        n_dof = self.num_actions
        dof_pos = self.data.qpos[7:7+n_dof]
        dof_vel = self.data.qvel[6:6+n_dof]
        quat = self.data.qpos[3:7]
        ang_vel = self.data.qvel[3:6]
        sim_torque = self.data.ctrl
        return dof_pos, dof_vel, quat, ang_vel, sim_torque

    def run(self):
        """Main simulation loop"""
        print("Starting TWIST2 23DOF simulation...")

        # Video recording setup
        if self.record_video:
            import imageio
            mp4_writer = imageio.get_writer('twist2_23dof_simulation.mp4', fps=30)
        else:
            mp4_writer = None

        self.reset_sim()
        self.reset(self.mujoco_default_dof_pos)

        steps = int(self.sim_duration / self.sim_dt)
        pbar = tqdm(range(steps), desc="Simulating TWIST2 23DOF...")

        # Send initial proprio to redis
        initial_obs = np.zeros(self.n_obs_single, dtype=np.float32)
        self.redis_pipeline.set("state_body_unitree_g1_23dof", json.dumps(initial_obs.tolist()))
        self.redis_pipeline.execute()

        measure_fps = self.measure_fps
        fps_measurements = []
        fps_iteration_count = 0
        fps_measurement_target = 1000
        last_policy_time = None

        policy_execution_times = []
        policy_step_count = 0
        policy_fps_print_interval = 100

        try:
            for i in pbar:
                t_start = time.time()
                dof_pos, dof_vel, quat, ang_vel, sim_torque = self.extract_data()
                
                if i % self.sim_decimation == 0:
                    # Build proprioceptive observation
                    rpy = quatToEuler(quat)
                    obs_body_dof_vel = dof_vel.copy()
                    obs_body_dof_vel[self.ankle_idx] = 0.
                    obs_proprio = np.concatenate([
                        ang_vel * 0.25,
                        rpy[:2],  # roll and pitch
                        (dof_pos - self.default_dof_pos),
                        obs_body_dof_vel * 0.05,
                        self.last_action
                    ])

                    state_body = np.concatenate([
                        ang_vel,
                        rpy[:2],
                        dof_pos])  # 3+2+23 = 28 dims

                    # Send proprio to redis
                    self.redis_pipeline.set("state_body_unitree_g1_23dof", json.dumps(state_body.tolist()))
                    self.redis_pipeline.set("t_state", int(time.time() * 1000))
                    self.redis_pipeline.execute()

                    # Get mimic obs from Redis
                    keys = ["action_body_unitree_g1_23dof"]
                    for key in keys:
                        self.redis_pipeline.get(key)
                    redis_results = self.redis_pipeline.execute()
                    
                    if redis_results[0] is not None:
                        action_mimic = json.loads(redis_results[0])
                    else:
                        # Default mimic obs if not available
                        action_mimic = np.zeros(self.n_mimic_obs, dtype=np.float32).tolist()

                    # Construct observation for TWIST2 controller
                    obs_full = np.concatenate([action_mimic, obs_proprio])
                    # Update history
                    obs_hist = np.array(self.proprio_history_buf).flatten()
                    self.proprio_history_buf.append(obs_full)
                    future_obs = np.array(action_mimic).copy()
                    # Combine all observations
                    obs_buf = np.concatenate([obs_full, obs_hist, future_obs])
                    
                    # Ensure correct total observation size
                    assert obs_buf.shape[0] == self.total_obs_size, f"Expected {self.total_obs_size} obs, got {obs_buf.shape[0]}"
                    
                    # Run policy
                    obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to(self.device)
                    with torch.no_grad():
                        raw_action = self.policy(obs_tensor).cpu().numpy().squeeze()
                    
                    # Track policy FPS
                    current_time = time.time()
                    if last_policy_time is not None:
                        policy_interval = current_time - last_policy_time
                        policy_execution_times.append(policy_interval)
                        policy_step_count += 1
                        
                        if policy_step_count % policy_fps_print_interval == 0:
                            recent_intervals = policy_execution_times[-policy_fps_print_interval:]
                            avg_interval = np.mean(recent_intervals)
                            avg_execution_fps = 1.0 / avg_interval
                            print(f"Policy Execution FPS (last {policy_fps_print_interval} steps): {avg_execution_fps:.2f} Hz")
                    
                    last_policy_time = current_time
                    
                    self.last_action = raw_action
                    raw_action = np.clip(raw_action, -10., 10.)
                    scaled_actions = raw_action * self.action_scale
                    pd_target = scaled_actions + self.default_dof_pos

                    # Update camera
                    pelvis_pos = self.data.xpos[self.model.body("pelvis").id]
                    self.viewer.cam.lookat = pelvis_pos
                    self.viewer.sync()
                    
                    if mp4_writer is not None:
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        mujoco.mjr_render(mujoco.MjrRect(0, 0, 640, 480), 
                                         self.viewer.scn, self.viewer.ctx)
                        mp4_writer.append_data(frame)

                # PD control
                torque = self.stiffness * (pd_target - dof_pos) - self.damping * dof_vel
                torque = np.clip(torque, -self.torque_limits, self.torque_limits)
                self.data.ctrl[:self.num_actions] = torque

                mujoco.mj_step(self.model, self.data)

                # FPS limiting
                if self.limit_fps:
                    t_elapsed = time.time() - t_start
                    t_sleep = self.sim_dt - t_elapsed
                    if t_sleep > 0:
                        time.sleep(t_sleep)

        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        finally:
            if mp4_writer is not None:
                mp4_writer.close()
            self.viewer.close()


def main():
    parser = argparse.ArgumentParser(description="TWIST2 23DOF G1 Simulation")
    parser.add_argument('--xml', type=str, 
                       default='../assets/g1/g1_sim2sim_23dof.xml',
                       help='MuJoCo XML file')
    parser.add_argument('--policy', type=str, required=True,
                       help='Path to ONNX policy file')
    parser.add_argument('--device', type=str, default='cuda',
                       help='Device for policy inference')
    parser.add_argument('--record_video', action='store_true',
                       help='Record simulation video')
    parser.add_argument('--measure_fps', action='store_true',
                       help='Measure and print FPS statistics')
    parser.add_argument('--no_limit_fps', action='store_true',
                       help='Disable FPS limiting')
    parser.add_argument('--policy_frequency', type=int, default=50,
                       help='Policy execution frequency in Hz')
    
    args = parser.parse_args()
    
    controller = RealTimePolicyController23DOF(
        xml_file=args.xml,
        policy_path=args.policy,
        device=args.device,
        record_video=args.record_video,
        measure_fps=args.measure_fps,
        limit_fps=not args.no_limit_fps,
        policy_frequency=args.policy_frequency,
    )
    
    controller.run()


if __name__ == '__main__':
    main()


