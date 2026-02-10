#!/bin/bash
# 環境チェックスクリプト
# 使い方: bash scripts/check_env.sh
#
# 別のPCとの挙動の違いを調べるために、環境情報を出力します。
# 両方のPCで実行して出力を比較してください。

set -e

echo "============================================"
echo "  TWIST2 23DOF Environment Check"
echo "  $(date)"
echo "  Hostname: $(hostname)"
echo "============================================"

# Conda環境
source ~/miniconda3/bin/activate twist2 2>/dev/null || source ~/anaconda3/bin/activate twist2 2>/dev/null || echo "WARNING: twist2 env not found"
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH 2>/dev/null

cd "$(dirname "$0")/.."

echo ""
echo "=== 1. Git Commit ==="
git log --oneline -1
git diff --stat HEAD

echo ""
echo "=== 2. Model Hashes ==="
md5sum assets/ckpts/g1_23dof_trained.onnx 2>/dev/null || echo "g1_23dof_trained.onnx NOT FOUND"
md5sum trained_model/model_0.pt 2>/dev/null || echo "trained_model/model_0.pt NOT FOUND"

echo ""
echo "=== 3. Package Versions ==="
python -c "
import sys
print(f'Python:     {sys.version.split()[0]}')
try:
    import mujoco; print(f'MuJoCo:     {mujoco.__version__}')
except: print('MuJoCo:     NOT INSTALLED')
try:
    import numpy; print(f'NumPy:      {numpy.__version__}')
except: print('NumPy:      NOT INSTALLED')
try:
    import onnxruntime; print(f'ONNX RT:    {onnxruntime.__version__}')
except: print('ONNX RT:    NOT INSTALLED')
try:
    import torch; print(f'PyTorch:    {torch.__version__}')
    print(f'CUDA avail: {torch.cuda.is_available()}')
    if torch.cuda.is_available():
        print(f'GPU:        {torch.cuda.get_device_name(0)}')
except: print('PyTorch:    NOT INSTALLED')
try:
    import scipy; print(f'SciPy:      {scipy.__version__}')
except: print('SciPy:      NOT INSTALLED')
try:
    import redis; print(f'Redis-py:   {redis.__version__}')
except: print('Redis-py:   NOT INSTALLED')
"

echo ""
echo "=== 4. Redis Status ==="
redis-cli ping 2>/dev/null && echo "Redis: RUNNING" || echo "Redis: NOT RUNNING"
redis-cli GET action_body_unitree_g1_23dof 2>/dev/null | python -c "
import sys, json
d = sys.stdin.read().strip()
if d and d != '(nil)':
    try:
        v = json.loads(d)
        print(f'Redis data: {len(v)} dims, values[0:6]={[round(x,4) for x in v[:6]]}')
    except:
        print(f'Redis data: raw={d[:50]}')
else:
    print('Redis data: EMPTY')
" 2>/dev/null || echo "Redis data: CANNOT READ"

echo ""
echo "=== 5. ONNX Model Info ==="
python -c "
import onnxruntime as ort
sess = ort.InferenceSession('assets/ckpts/g1_23dof_trained.onnx', providers=['CPUExecutionProvider'])
inp = sess.get_inputs()[0]
out = sess.get_outputs()[0]
print(f'Input:  {inp.shape} ({inp.shape[1]} dims)')
print(f'Output: {out.shape} ({out.shape[1]} actions)')
print(f'Provider: {sess.get_providers()}')

# Test inference with zeros
import numpy as np
test_input = np.zeros((1, inp.shape[1]), dtype=np.float32)
test_output = sess.run(None, {inp.name: test_input})[0]
print(f'Zero-input output[0:5]: {[round(float(x),6) for x in test_output[0,:5]]}')
print(f'Zero-input output sum:  {round(float(test_output.sum()),6)}')
" 2>/dev/null || echo "ONNX model check FAILED"

echo ""
echo "=== 6. MuJoCo XML Check ==="
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('assets/g1/g1_sim2sim_23dof.xml')
print(f'nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody}')
print(f'timestep={m.opt.timestep}')
" 2>/dev/null || echo "MuJoCo XML check FAILED"

echo ""
echo "=== 7. OS / CPU ==="
uname -a
lscpu | grep "Model name" || echo "CPU info unavailable"

echo ""
echo "============================================"
echo "  Check complete. Compare outputs between PCs."
echo "============================================"

