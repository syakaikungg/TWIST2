#!/bin/bash
# RunPod 学習実行スクリプト
# tmux内で実行推奨:
#   tmux new -s train
#   bash /workspace/TWIST2/scripts/runpod_train.sh
#   # Ctrl+B, D でデタッチ
set -e

# Conda activate
eval "$(conda shell.bash hook)"
conda activate twist2
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

cd /workspace/TWIST2/legged_gym/legged_gym/scripts

echo "=== TWIST2 23DOF 学習開始 ==="
echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader)"
echo ""

# Step 1: Teacher学習
echo "=== Step 1: Teacher (Privileged) Policy ==="
echo "開始時刻: $(date)"
python train.py \
    --task g1_23dof_priv \
    --proj_name g1_23dof \
    --exptid exp1_teacher \
    --num_envs 4096 \
    --headless \
    --max_iterations 25000 \
    --no_wandb

echo "Teacher完了: $(date)"
echo ""

# Step 2: Student蒸留学習
echo "=== Step 2: Student (Future) Policy with DAgger ==="
echo "開始時刻: $(date)"
python train.py \
    --task g1_23dof_future \
    --proj_name g1_23dof \
    --exptid exp1_student \
    --num_envs 4096 \
    --headless \
    --max_iterations 30000 \
    --no_wandb \
    --teacher_exptid exp1_teacher

echo "Student完了: $(date)"
echo ""
echo "=== 学習完了! ==="
echo "チェックポイント: /workspace/TWIST2/logs/"
echo ""
echo "ローカルPCへのダウンロード:"
echo "  scp -P <port> -r root@<host>:/workspace/TWIST2/logs/ ~/workspace/sakai/TWIST2/TWIST2/logs/"


