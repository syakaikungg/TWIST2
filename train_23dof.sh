#!/bin/bash
# TWIST2 23DOF G1 Training Script
#
# 使用方法:
#   ./train_23dof.sh [experiment_name] [cuda_device]
#
# 例:
#   ./train_23dof.sh g1_23dof_exp1 cuda:0
#
# ステップ:
# 1. まずデータセット変換スクリプトを実行 (初回のみ)
# 2. Teacher (privileged) ポリシーを学習
# 3. Student (future) ポリシーを蒸留学習

set -e

# デフォルト値
EXP_NAME=${1:-"g1_23dof_exp"}
CUDA_DEVICE=${2:-"cuda:0"}

echo "=========================================="
echo "TWIST2 23DOF G1 Training"
echo "Experiment: $EXP_NAME"
echo "Device: $CUDA_DEVICE"
echo "=========================================="

cd "$(dirname "$0")"

# データセットが変換されているか確認
MOTION_DATA_DIR="motion_data_23dof"
if [ ! -d "$MOTION_DATA_DIR" ] || [ -z "$(ls -A $MOTION_DATA_DIR 2>/dev/null)" ]; then
    echo ""
    echo "[WARNING] 23DOF motion data not found!"
    echo ""
    echo "Please run the conversion script first:"
    echo "  python scripts/convert_29dof_to_23dof.py \\"
    echo "      --yaml legged_gym/motion_data_configs/twist2_dataset.yaml \\"
    echo "      --output_dir motion_data_23dof"
    echo ""
    echo "Then update the dataset YAML:"
    echo "  legged_gym/motion_data_configs/twist2_23dof_dataset.yaml"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Conda環境を確認
if [ -z "$CONDA_DEFAULT_ENV" ] || [ "$CONDA_DEFAULT_ENV" != "twist2" ]; then
    echo "Activating conda environment 'twist2'..."
    source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null
    conda activate twist2
fi

echo ""
echo "Step 1: Training Teacher (Privileged) Policy"
echo "============================================="

cd legged_gym/legged_gym/scripts

# Teacher学習
python train.py \
    --task g1_23dof_priv \
    --run_name "${EXP_NAME}_teacher" \
    --num_envs 2048 \
    --device "$CUDA_DEVICE" \
    --max_iterations 25000

echo ""
echo "Step 2: Training Student (Future) Policy with DAgger"
echo "====================================================="

# Student蒸留学習
# Teacher checkpointを自動検出
TEACHER_CKPT=$(ls -t ../../../logs/g1_23dof_priv/${EXP_NAME}_teacher/model_*.pt 2>/dev/null | head -1)

if [ -z "$TEACHER_CKPT" ]; then
    echo "Error: Teacher checkpoint not found"
    exit 1
fi

echo "Using teacher checkpoint: $TEACHER_CKPT"

python train.py \
    --task g1_23dof_future \
    --run_name "${EXP_NAME}_student" \
    --num_envs 2048 \
    --device "$CUDA_DEVICE" \
    --max_iterations 30000 \
    --resume_path "$TEACHER_CKPT"

echo ""
echo "Training Complete!"
echo "==================="
echo "Teacher checkpoint: logs/g1_23dof_priv/${EXP_NAME}_teacher/"
echo "Student checkpoint: logs/g1_23dof_future/${EXP_NAME}_student/"
echo ""
echo "To export ONNX model:"
echo "  python save_onnx.py --task g1_23dof_future --run_name ${EXP_NAME}_student"


