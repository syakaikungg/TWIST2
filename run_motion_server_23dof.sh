#!/bin/bash
# 23DOF用モーション配信スクリプト
# 使い方: bash run_motion_server_23dof.sh [motion_file]

source ~/miniconda3/bin/activate twist2
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

script_dir=$(dirname $(realpath $0))
motion_file=${1:-"${script_dir}/assets/example_motions/0807_yanjie_walk_001.pkl"}

cd "${script_dir}/deploy_real"

CUDA_VISIBLE_DEVICES="" python -c "
import sys; m=type(sys)('isaacgym'); m.torch_utils=type(sys)('t'); sys.modules['isaacgym']=m; sys.modules['isaacgym.torch_utils']=m.torch_utils

import time, redis, json, numpy as np, torch
from pose.utils.motion_lib_pkl import MotionLib
from data_utils.rot_utils import euler_from_quaternion_torch, quat_rotate_inverse_torch
from data_utils.params import DEFAULT_MIMIC_OBS

motion_file = '${motion_file}'
motion_lib = MotionLib(motion_file, device='cpu')
total_len = motion_lib.get_motion_length(torch.tensor([0])).item()
print(f'Motion: {motion_file}')
print(f'Length: {total_len:.1f}s')

# 29DOF -> 23DOF index mapping
KEEP = [i for i in range(29) if i not in [13, 14, 19, 20, 26, 27]]

r = redis.Redis()
robot_key = 'unitree_g1_23dof'
control_dt = 0.02
motion_id = torch.zeros(1, dtype=torch.int)

# 23DOF default standing
default_23 = np.concatenate([
    np.array([0, 0, 0.793, 0, 0, 0]),
    np.array([-0.2, 0, 0, 0.4, -0.2, 0, -0.2, 0, 0, 0.4, -0.2, 0, 0, 0, 0.4, 0, 1.2, 0, 0, -0.4, 0, 1.2, 0])
])

# Phase 1: Stand 3s
print('Standing (3s)...')
for i in range(150):
    r.set(f'action_body_{robot_key}', json.dumps(default_23.tolist()))
    time.sleep(control_dt)

# Phase 2: Walk loop
print('Walking (loop, Ctrl+C to stop)...')
t = 0.0
try:
    while True:
        mt = torch.tensor([t % total_len])
        rp, rr, rv, rav, dp, dv, lbp, _, _ = motion_lib.calc_motion_frame(motion_id, mt)
        roll, pitch, yaw = euler_from_quaternion_torch(rr, scalar_first=False)
        rvl = quat_rotate_inverse_torch(rr, rv, scalar_first=False)
        ravl = quat_rotate_inverse_torch(rr, rav, scalar_first=False)
        dof_23 = dp.flatten().numpy()[KEEP]
        obs = np.concatenate([rvl[0,:2].numpy(), rp[0,2:3].numpy(), roll.reshape(-1)[:1].numpy(), pitch.reshape(-1)[:1].numpy(), ravl[0,2:3].numpy(), dof_23])
        r.set(f'action_body_{robot_key}', json.dumps(obs.tolist()))
        t += control_dt
        if int(t/control_dt) % 250 == 0:
            print(f't={t:.1f}s / {total_len:.1f}s (loop {int(t/total_len)})')
        time.sleep(control_dt)
except KeyboardInterrupt:
    print('Stopping... returning to stand')
    for i in range(100):
        r.set(f'action_body_{robot_key}', json.dumps(default_23.tolist()))
        time.sleep(control_dt)
    print('Done.')
"

