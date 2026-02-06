"""
G1 23DOF Configuration
- 腰: waist_yaw のみ (waist_roll, waist_pitch は固定)
- 腕: shoulder×3 + elbow + wrist_roll (wrist_pitch, wrist_yaw は固定)
- 脚: 6DOF×2 (変更なし)

関節順序 (23DOF):
[0-5]:   左脚 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
[6-11]:  右脚 (同上)
[12]:    腰 (waist_yaw)
[13-17]: 左腕 (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
[18-22]: 右腕 (同上)
"""

from legged_gym.envs.g1.g1_mimic_distill_config import G1MimicPrivCfg, G1MimicPrivCfgPPO
from legged_gym.envs.base.humanoid_mimic_config import HumanoidMimicCfgPPO
from legged_gym import LEGGED_GYM_ROOT_DIR


# 23DOF用の設定
NUM_ACTIONS_23DOF = 23
TAR_MOTION_STEPS_FUTURE = [0]


class G1_23DOF_MimicPrivCfg(G1MimicPrivCfg):
    """23DOF G1用のプライベート設定"""
    
    class env(G1MimicPrivCfg.env):
        tar_motion_steps_priv = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45,
                                 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,]
        tar_motion_steps = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45,
                            50, 55, 60, 65, 70, 75, 80, 85, 90, 95,]
        
        num_envs = 2048  # 16GB VRAM用に調整 (4096 → 2048)
        num_actions = NUM_ACTIONS_23DOF
        obs_type = 'priv'
        n_priv_latent = 4 + 1 + 2 * num_actions
        extra_critic_obs = 3
        n_priv = 0
        
        n_proprio = 3 + 2 + 3 * num_actions  # 3 + 2 + 69 = 74
        n_priv_mimic_obs = len(tar_motion_steps_priv) * (21 + num_actions + 3 * 9)
        n_mimic_obs_single = 6 + NUM_ACTIONS_23DOF  # 6 + 23 = 29
        n_mimic_obs = len(tar_motion_steps) * n_mimic_obs_single
        n_priv_info = 3 + 3 + 4 + 3 * 9 + 2 + 4 + 1 + 2 * num_actions
        history_len = 10
        
        n_obs_single = n_priv_mimic_obs + n_proprio + n_priv_info
        n_priv_obs_single = n_priv_mimic_obs + n_proprio + n_priv_info
        
        num_observations = n_priv_obs_single
        num_privileged_obs = n_priv_obs_single
        
        env_spacing = 3.
        send_timeouts = True
        episode_length_s = 10
        
        randomize_start_pos = True
        randomize_start_yaw = False
        
        history_encoding = True
        contact_buf_len = 10
        
        normalize_obs = True
        
        enable_early_termination = True
        pose_termination = True
        pose_termination_dist = 0.7
        rand_reset = True
        
        track_root = False
        root_tracking_termination_dist = 2.0
        
        # 23DOF用 dof_err_w
        dof_err_w = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # 左脚 (6)
                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # 右脚 (6)
                     1.0,                           # 腰 waist_yaw (1)
                     1.0, 1.0, 1.0, 1.0, 1.0,       # 左腕 (5)
                     1.0, 1.0, 1.0, 1.0, 1.0,       # 右腕 (5)
                     ]
        
        global_obs = False
    
    class terrain(G1MimicPrivCfg.terrain):
        mesh_type = 'plane'
        height = [0, 0.00]
        horizontal_scale = 0.1
    
    class init_state(G1MimicPrivCfg.init_state):
        pos = [0, 0, 1.0]
        # 23DOF用 default_joint_angles (waist_roll, waist_pitch, wrist_pitch, wrist_yaw を削除)
        default_joint_angles = {
            'left_hip_pitch_joint': -0.2,
            'left_hip_roll_joint': 0.0,
            'left_hip_yaw_joint': 0.0,
            'left_knee_joint': 0.4,
            'left_ankle_pitch_joint': -0.2,
            'left_ankle_roll_joint': 0.0,
            
            'right_hip_pitch_joint': -0.2,
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.4,
            'right_ankle_pitch_joint': -0.2,
            'right_ankle_roll_joint': 0.0,
            
            'waist_yaw_joint': 0.0,
            # waist_roll_joint, waist_pitch_joint は固定
            
            'left_shoulder_pitch_joint': 0.0,
            'left_shoulder_roll_joint': 0.4,
            'left_shoulder_yaw_joint': 0.0,
            'left_elbow_joint': 1.2,
            'left_wrist_roll_joint': 0.0,
            # left_wrist_pitch_joint, left_wrist_yaw_joint は固定
            
            'right_shoulder_pitch_joint': 0.0,
            'right_shoulder_roll_joint': -0.4,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 1.2,
            'right_wrist_roll_joint': 0.0,
            # right_wrist_pitch_joint, right_wrist_yaw_joint は固定
        }
    
    class control(G1MimicPrivCfg.control):
        stiffness = {'hip_yaw': 100,
                     'hip_roll': 100,
                     'hip_pitch': 100,
                     'knee': 150,
                     'ankle': 40,
                     'waist': 150,
                     'shoulder': 40,
                     'elbow': 40,
                     'wrist': 40,
                     }
        damping = {'hip_yaw': 2,
                   'hip_roll': 2,
                   'hip_pitch': 2,
                   'knee': 4,
                   'ankle': 2,
                   'waist': 4,
                   'shoulder': 5,
                   'elbow': 5,
                   'wrist': 5,
                   }
        action_scale = 0.5
        decimation = 10
    
    class sim(G1MimicPrivCfg.sim):
        dt = 0.002
    
    class normalization(G1MimicPrivCfg.normalization):
        clip_actions = 5.0
    
    class asset(G1MimicPrivCfg.asset):
        # 23DOF用 URDF
        file = f'{LEGGED_GYM_ROOT_DIR}/../assets/g1/g1_custom_collision_23dof.urdf'
        
        torso_name: str = 'pelvis'
        chest_name: str = 'imu_in_torso'
        thigh_name: str = 'hip'
        shank_name: str = 'knee'
        foot_name: str = 'ankle_roll_link'
        waist_name: list = ['torso_link', 'waist_roll_link', 'waist_yaw_link']
        upper_arm_name: str = 'shoulder_roll_link'
        lower_arm_name: str = 'elbow_link'
        hand_name: list = ['right_rubber_hand', 'left_rubber_hand']
        
        feet_bodies = ['left_ankle_roll_link', 'right_ankle_roll_link']
        n_lower_body_dofs: int = 12
        
        penalize_contacts_on = ["shoulder", "elbow", "hip", "knee"]
        terminate_after_contacts_on = []
        
        # 23DOF用 dof_armature (waist_roll/pitch と wrist_pitch/yaw を削除)
        # 脚12 + 腰1 + 腕10 = 23
        dof_armature = [0.0103, 0.0251, 0.0103, 0.0251, 0.003597, 0.003597] * 2 + \
                       [0.0103] + \
                       [0.003597, 0.003597, 0.003597, 0.003597, 0.003597] * 2
        
        collapse_fixed_joints = False
    
    class rewards(G1MimicPrivCfg.rewards):
        regularization_names = []
        regularization_scale = 1.0
        regularization_scale_range = [0.8, 2.0]
        regularization_scale_curriculum = False
        regularization_scale_gamma = 0.0001
        
        class scales:
            tracking_joint_dof = 2.0
            tracking_joint_vel = 0.2
            tracking_root_translation_z = 1.0
            tracking_root_rotation = 1.0
            tracking_root_linear_vel = 1.0
            tracking_root_angular_vel = 1.0
            tracking_keybody_pos = 2.0
            tracking_keybody_pos_global = 2.0
            alive = 0.5
            feet_slip = -0.1
            feet_contact_forces = -5e-4
            feet_stumble = -1.25
            dof_pos_limits = -5.0
            dof_torque_limits = -1.0
            dof_vel = -1e-4
            dof_acc = -5e-8
            action_rate = -0.01
            feet_air_time = 5.0
            ang_vel_xy = -0.01
            ankle_dof_acc = -5e-8 * 2
            ankle_dof_vel = -1e-4 * 2
        
        min_dist = 0.1
        max_dist = 0.4
        max_knee_dist = 0.4
        feet_height_target = 0.2
        feet_air_time_target = 0.5
        only_positive_rewards = False
        tracking_sigma = 0.2
        tracking_sigma_ang = 0.125
        max_contact_force = 500
        soft_torque_limit = 0.95
        torque_safety_limit = 0.9
        termination_roll = 4.0
        termination_pitch = 4.0
        root_height_diff_threshold = 0.3
    
    class evaluations:
        tracking_joint_dof = True
        tracking_joint_vel = True
        tracking_root_translation = True
        tracking_root_rotation = True
        tracking_root_vel = True
        tracking_root_ang_vel = True
        tracking_keybody_pos = True
        tracking_root_pose_delta_local = True
        tracking_root_rotation_delta_local = True
    
    class domain_rand:
        domain_rand_general = True
        
        randomize_gravity = (True and domain_rand_general)
        gravity_rand_interval_s = 4
        gravity_range = (-0.1, 0.1)
        
        randomize_friction = (True and domain_rand_general)
        friction_range = [0.1, 2.]
        
        randomize_base_mass = (True and domain_rand_general)
        added_mass_range = [-3., 3]
        
        randomize_base_com = (True and domain_rand_general)
        added_com_range = [-0.05, 0.05]
        
        push_robots = (True and domain_rand_general)
        push_interval_s = 4
        max_push_vel_xy = 1.0
        
        push_end_effector = (False and domain_rand_general)
        push_end_effector_interval_s = 2
        max_push_force_end_effector = 10.0
        
        randomize_motor = (True and domain_rand_general)
        motor_strength_range = [0.8, 1.2]
        
        action_delay = (True and domain_rand_general)
        action_buf_len = 8
    
    class noise(G1MimicPrivCfg.noise):
        add_noise = True
        noise_increasing_steps = 50_000
        
        class noise_scales:
            dof_pos = 0.01
            dof_vel = 0.1
            lin_vel = 0.1
            ang_vel = 0.1
            gravity = 0.05
            imu = 0.1
    
    class motion(G1MimicPrivCfg.motion):
        motion_curriculum = True
        motion_curriculum_gamma = 0.01
        reset_consec_frames = 30
        key_bodies = ["left_rubber_hand", "right_rubber_hand", 
                      "left_ankle_roll_link", "right_ankle_roll_link", 
                      "left_knee_link", "right_knee_link", 
                      "left_elbow_link", "right_elbow_link", "head_mocap"]
        upper_key_bodies = ["left_rubber_hand", "right_rubber_hand", 
                           "left_elbow_link", "right_elbow_link", "head_mocap"]
        sample_ratio = 1.0
        motion_smooth = True
        motion_decompose = False
        
        # 23DOF用データセット（後で変換スクリプトで作成）
        motion_file = f"{LEGGED_GYM_ROOT_DIR}/motion_data_configs/twist2_23dof_dataset.yaml"


class G1_23DOF_MimicStuFutureCfg(G1_23DOF_MimicPrivCfg):
    """23DOF用 Future Student Policy Config"""
    
    class env(G1_23DOF_MimicPrivCfg.env):
        obs_type = 'student_future'
        tar_motion_steps = [0]
        tar_motion_steps_future = TAR_MOTION_STEPS_FUTURE
        
        n_mimic_obs_single = 6 + NUM_ACTIONS_23DOF  # 29
        n_mimic_obs = len(tar_motion_steps) * n_mimic_obs_single
        n_proprio = G1_23DOF_MimicPrivCfg.env.n_proprio  # 74
        
        n_future_obs_single = 6 + NUM_ACTIONS_23DOF  # 29
        n_future_obs = len(tar_motion_steps_future) * n_future_obs_single
        
        n_obs_single = n_mimic_obs + n_proprio
        num_observations = n_obs_single * (G1_23DOF_MimicPrivCfg.env.history_len + 1) + n_future_obs
        
        enable_force_curriculum = False
    
    class motion(G1_23DOF_MimicPrivCfg.motion):
        motion_file = f"{LEGGED_GYM_ROOT_DIR}/motion_data_configs/twist2_23dof_dataset.yaml"
        motion_curriculum = True
        motion_curriculum_gamma = 0.01
        motion_decompose = False
        motion_dr_enabled = False


class G1_23DOF_MimicPrivCfgPPO(HumanoidMimicCfgPPO):
    """23DOF用 PPO Config"""
    seed = 1
    
    class runner(HumanoidMimicCfgPPO.runner):
        policy_class_name = 'ActorCriticMimic'
        algorithm_class_name = 'PPO'
        runner_class_name = 'OnPolicyRunnerMimic'
        max_iterations = 1_000_002
        
        save_interval = 500
        experiment_name = 'g1_23dof'
        run_name = ''
        resume = False
        load_run = -1
        checkpoint = -1
        resume_path = None
    
    class algorithm(HumanoidMimicCfgPPO.algorithm):
        grad_penalty_coef_schedule = [0.00, 0.00, 700, 1000]
        std_schedule = [1.0, 0.4, 4000, 1500]
        entropy_coef = 0.005
    
    class policy(HumanoidMimicCfgPPO.policy):
        # 23DOF用 action_std: 脚12 + 腰1 + 腕10
        action_std = [0.7] * 12 + [0.4] * 1 + [0.5] * 10
        init_noise_std = 1.0
        obs_context_len = 11
        actor_hidden_dims = [512, 512, 256, 128]
        critic_hidden_dims = [512, 512, 256, 128]
        activation = 'silu'
        layer_norm = True
        motion_latent_dim = 128


class G1_23DOF_MimicStuFutureCfgDAgger(G1_23DOF_MimicStuFutureCfg):
    """23DOF用 DAgger Training Config"""
    
    seed = 1
    
    class teachercfg(G1_23DOF_MimicPrivCfgPPO):
        pass
    
    class runner(G1_23DOF_MimicPrivCfgPPO.runner):
        policy_class_name = 'ActorCriticFuture'
        algorithm_class_name = 'DaggerPPO'
        runner_class_name = 'OnPolicyDaggerRunner'
        max_iterations = 30_001
        warm_iters = 100
        
        save_interval = 500
        experiment_name = 'g1_23dof'
        run_name = ''
        resume = False
        load_run = -1
        checkpoint = -1
        resume_path = None
        
        teacher_experiment_name = 'g1_23dof'
        teacher_proj_name = 'g1_23dof_priv_mimic'
        teacher_checkpoint = -1
        eval_student = False
        save_to_wandb = False
    
    class algorithm(HumanoidMimicCfgPPO.algorithm):
        grad_penalty_coef_schedule = [0.00, 0.00, 700, 1000]
        std_schedule = [1.0, 0.4, 4000, 1500]
        entropy_coef = 0.005
        
        dagger_coef_anneal_steps = 60000
        dagger_coef = 0.2
        dagger_coef_min = 0.1
        
        future_weight_decay = 0.95
        future_consistency_loss = 0.1
    
    class policy(HumanoidMimicCfgPPO.policy):
        # 23DOF用 action_std
        action_std = [0.7] * 12 + [0.4] * 1 + [0.5] * 10
        init_noise_std = 1.0
        obs_context_len = 11
        actor_hidden_dims = [512, 512, 256, 128]
        critic_hidden_dims = [512, 512, 256, 128]
        activation = 'silu'
        layer_norm = True
        motion_latent_dim = 128
        
        future_encoder_dims = [256, 256, 128]
        future_attention_heads = 4
        future_dropout = 0.1
        temporal_embedding_dim = 64
        future_latent_dim = 128
        num_future_steps = len(TAR_MOTION_STEPS_FUTURE)
        
        num_future_observations = G1_23DOF_MimicStuFutureCfg.env.n_future_obs
        
        num_experts = 4
        expert_hidden_dims = [256, 128]
        gating_hidden_dim = 128
        moe_temperature = 1.0
        moe_topk = None
        load_balancing_loss_weight = 0.01

