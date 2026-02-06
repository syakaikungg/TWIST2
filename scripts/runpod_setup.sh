#!/bin/bash
# RunPod 環境構築スクリプト
# RunPodにSSH接続後に実行:
#   bash /workspace/TWIST2/scripts/runpod_setup.sh
set -e

echo "=== RunPod TWIST2 環境構築 ==="

cd /workspace

# 1. コード展開
echo "[1/6] コード展開..."
if [ ! -d "TWIST2" ]; then
    mkdir TWIST2 && tar xzf twist2_code.tar.gz -C TWIST2
fi

echo "[2/6] モーションデータ展開..."
if [ ! -d "TWIST2/motion_data_23dof" ] || [ -z "$(ls -A TWIST2/motion_data_23dof 2>/dev/null)" ]; then
    mkdir -p TWIST2/motion_data_23dof && tar xzf twist2_motion_23dof.tar.gz -C TWIST2/motion_data_23dof
fi

echo "[3/6] IsaacGym展開..."
if [ ! -d "isaacgym" ]; then
    tar xzf isaacgym.tar.gz
fi

# 2. Conda環境
echo "[4/6] Conda環境セットアップ..."
# condaが存在するか確認
if command -v conda &> /dev/null; then
    echo "Conda found"
else
    echo "Installing Miniconda..."
    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
    bash /tmp/miniconda.sh -b -p /root/miniconda3
    eval "$(/root/miniconda3/bin/conda shell.bash hook)"
    conda init bash
    source ~/.bashrc
fi

# twist2環境が存在しなければ作成
if ! conda env list | grep -q "twist2"; then
    conda create -n twist2 python=3.8 -y
fi

# conda activate (スクリプト内では source が必要)
eval "$(conda shell.bash hook)"
conda activate twist2

# 3. IsaacGymインストール
echo "[5/6] IsaacGym + 依存関係インストール..."
cd /workspace/isaacgym/python
pip install -e . -q 2>&1 | tail -3

# 4. TWIST2依存関係
cd /workspace/TWIST2
cd rsl_rl && pip install -e . -q 2>&1 | tail -1 && cd ..
cd legged_gym && pip install -e . -q 2>&1 | tail -1 && cd ..
cd pose && pip install -e . -q 2>&1 | tail -1 && cd ..

pip install "numpy==1.23.0" pydelatin wandb tqdm opencv-python mujoco rich \
    termcolor pyfqmr ipdb flask dill gdown hydra-core imageio \
    pytorch-kinematics zmq redis isaacgym-stubs -q 2>&1 | tail -3

# 5. YAMLパス更新
echo "[6/6] データセットパス更新..."
sed -i 's|root_path:.*|root_path: /workspace/TWIST2/motion_data_23dof|' \
    legged_gym/motion_data_configs/twist2_23dof_dataset.yaml

# 6. LD_LIBRARY_PATH設定
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH' > $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh

# 7. 動作確認
echo ""
echo "=== 動作確認 ==="
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
python -c "import isaacgym; print('IsaacGym: OK')"
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}'); print(f'GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

echo ""
echo "=== セットアップ完了 ==="
echo ""
echo "学習を開始するには:"
echo "  bash /workspace/TWIST2/scripts/runpod_train.sh"
echo ""
echo "tmuxで実行する場合（SSH切断しても学習継続）:"
echo "  tmux new -s train"
echo "  bash /workspace/TWIST2/scripts/runpod_train.sh"
echo "  # Ctrl+B, D でデタッチ"

