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
# 3. Student (future) ポリシーを蒸留学習 (DAgger)

set -e

# デフォルト値
EXP_NAME=${1:-"g1_23dof_exp"}
CUDA_DEVICE=${2:-"cuda:0"}

# proj_name: Teacher学習のログディレクトリ名
# DAgger config (G1_23DOF_MimicStuFutureCfgDAgger) の teacher_proj_name と一致させる
TEACHER_PROJ_NAME="g1_23dof_priv_mimic"
STUDENT_PROJ_NAME="g1_23dof_future"

echo "=========================================="
echo "TWIST2 23DOF G1 Training"
echo "Experiment: $EXP_NAME"
echo "Device: $CUDA_DEVICE"
echo "Teacher proj: $TEACHER_PROJ_NAME"
echo "Student proj: $STUDENT_PROJ_NAME"
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
echo "  task:      g1_23dof_priv"
echo "  proj_name: $TEACHER_PROJ_NAME"
echo "  exptid:    ${EXP_NAME}_teacher"
echo ""

cd legged_gym/legged_gym/scripts

# Teacher学習
# train.py は log_pth = LEGGED_GYM_ROOT_DIR/logs/<proj_name>/<exptid> でチェックポイントを保存
python train.py \
    --task g1_23dof_priv \
    --proj_name "$TEACHER_PROJ_NAME" \
    --exptid "${EXP_NAME}_teacher" \
    --num_envs 2048 \
    --device "$CUDA_DEVICE" \
    --max_iterations 5000 \
    --no_wandb  

echo ""
echo "Step 2: Training Student (Future) Policy with DAgger"
echo "====================================================="
echo "  task:           g1_23dof_future"
echo "  proj_name:      $STUDENT_PROJ_NAME"
echo "  exptid:         ${EXP_NAME}_student"
echo "  teacher_exptid: ${EXP_NAME}_teacher"
echo ""

# Student蒸留学習 (DAgger)
# DAggerランナーは teacher_proj_name (config: 'g1_23dof_priv_mimic') と
# teacher_experiment_name (--teacher_exptid で上書き) から自動でTeacherを探索:
#   logs/<teacher_proj_name>/<teacher_experiment_name>/model_*.pt
python train.py \
    --task g1_23dof_future \
    --proj_name "$STUDENT_PROJ_NAME" \
    --exptid "${EXP_NAME}_student" \
    --num_envs 2048 \
    --device "$CUDA_DEVICE" \
    --max_iterations 6000 \
    --teacher_exptid "${EXP_NAME}_teacher" \
    --no_wandb

echo ""
echo "Training Complete!"
echo "==================="
echo "Teacher checkpoint: logs/$TEACHER_PROJ_NAME/${EXP_NAME}_teacher/"
echo "Student checkpoint: logs/$STUDENT_PROJ_NAME/${EXP_NAME}_student/"
echo ""
echo "To export ONNX model:"
echo "  cd legged_gym/legged_gym/scripts"
echo "  python save_onnx.py --task g1_23dof_future --proj_name $STUDENT_PROJ_NAME --exptid ${EXP_NAME}_student"
