#!/usr/bin/env python3
"""
29DOFモーションデータを23DOFに変換するスクリプト

使用方法:
    python convert_29dof_to_23dof.py --input_dir /path/to/29dof/motions --output_dir /path/to/23dof/motions
    
または既存のデータセットYAMLから変換:
    python convert_29dof_to_23dof.py --yaml /path/to/twist2_dataset.yaml --output_dir /path/to/23dof/motions

29DOF関節順序:
    [0-5]:   左脚 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
    [6-11]:  右脚
    [12]:    waist_yaw
    [13]:    waist_roll     ← 削除
    [14]:    waist_pitch    ← 削除
    [15-21]: 左腕 (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw)
    [22-28]: 右腕

23DOF関節順序:
    [0-5]:   左脚
    [6-11]:  右脚
    [12]:    waist_yaw
    [13-17]: 左腕 (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
    [18-22]: 右腕
"""

import os
import sys
import argparse
import pickle
import numpy as np
from pathlib import Path
from tqdm import tqdm
import yaml


# 29DOFから削除するインデックス
# waist_roll(13), waist_pitch(14), left_wrist_pitch(19), left_wrist_yaw(20), right_wrist_pitch(26), right_wrist_yaw(27)
REMOVE_INDICES_29_TO_23 = [13, 14, 19, 20, 26, 27]

# 29DOFから23DOFへのマッピング（削除するインデックスを除外）
KEEP_INDICES_29_TO_23 = [i for i in range(29) if i not in REMOVE_INDICES_29_TO_23]


def convert_motion_file(input_path: str, output_path: str) -> bool:
    """
    単一のモーションファイルを29DOFから23DOFに変換
    
    Args:
        input_path: 入力pklファイルパス
        output_path: 出力pklファイルパス
        
    Returns:
        成功した場合True
    """
    try:
        with open(input_path, 'rb') as f:
            data = pickle.load(f)
        
        # dof_posを変換 (T, 29) → (T, 23)
        if 'dof_pos' in data:
            dof_pos = data['dof_pos']
            if dof_pos.shape[-1] == 29:
                data['dof_pos'] = dof_pos[..., KEEP_INDICES_29_TO_23]
            elif dof_pos.shape[-1] == 23:
                pass  # 既に23DOF
            else:
                print(f"Warning: Unexpected dof_pos shape {dof_pos.shape} in {input_path}")
                return False
        
        # dof_velを変換 (T, 29) → (T, 23)
        if 'dof_vel' in data:
            dof_vel = data['dof_vel']
            if dof_vel.shape[-1] == 29:
                data['dof_vel'] = dof_vel[..., KEEP_INDICES_29_TO_23]
            elif dof_vel.shape[-1] == 23:
                pass
            else:
                print(f"Warning: Unexpected dof_vel shape {dof_vel.shape} in {input_path}")
                return False
        
        # メタデータを更新
        if 'num_dof' in data:
            data['num_dof'] = 23
        
        # 出力ディレクトリを作成
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # 保存
        with open(output_path, 'wb') as f:
            pickle.dump(data, f)
        
        return True
        
    except Exception as e:
        print(f"Error converting {input_path}: {e}")
        return False


def convert_directory(input_dir: str, output_dir: str) -> tuple:
    """
    ディレクトリ内の全モーションファイルを変換
    
    Returns:
        (成功数, 失敗数)
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    pkl_files = list(input_path.glob("**/*.pkl"))
    
    if not pkl_files:
        print(f"No .pkl files found in {input_dir}")
        return 0, 0
    
    success = 0
    failed = 0
    
    for pkl_file in tqdm(pkl_files, desc="Converting motions"):
        # 相対パスを維持
        relative_path = pkl_file.relative_to(input_path)
        out_file = output_path / relative_path
        
        if convert_motion_file(str(pkl_file), str(out_file)):
            success += 1
        else:
            failed += 1
    
    return success, failed


def convert_from_yaml(yaml_path: str, output_dir: str) -> tuple:
    """
    データセットYAMLに記載されている全モーションファイルを変換
    """
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # root_pathがYAMLに定義されていればそれを使用、なければyamlのディレクトリを使用
    base_dir = config.get('root_path', os.path.dirname(yaml_path))
    
    success = 0
    failed = 0
    
    # YAMLの形式に応じて処理
    if 'motions' in config:
        motions = config['motions']
    elif 'motion_files' in config:
        motions = config['motion_files']
    else:
        # ルートに直接モーションリストがある場合
        motions = config if isinstance(config, list) else []
    
    if not motions:
        print(f"No motions found in {yaml_path}")
        return 0, 0
    
    output_path = Path(output_dir)
    
    for motion_entry in tqdm(motions, desc="Converting motions from YAML"):
        if isinstance(motion_entry, dict):
            motion_file = motion_entry.get('file', '')
        else:
            motion_file = motion_entry
        
        if not motion_file:
            continue
        
        # 相対パスを絶対パスに変換
        if not os.path.isabs(motion_file):
            motion_file = os.path.join(base_dir, motion_file)
        
        if not os.path.exists(motion_file):
            print(f"Warning: Motion file not found: {motion_file}")
            failed += 1
            continue
        
        # 出力パスを決定（ディレクトリ構造を維持）
        relative_path = os.path.relpath(motion_file, base_dir)
        out_file = output_path / relative_path
        
        if convert_motion_file(motion_file, str(out_file)):
            success += 1
        else:
            failed += 1
    
    return success, failed


def create_23dof_yaml(input_yaml: str, output_yaml: str, output_motion_dir: str):
    """
    23DOF用のデータセットYAMLを作成
    """
    with open(input_yaml, 'r') as f:
        config = yaml.safe_load(f)
    
    # モーションファイルパスを更新
    if 'motions' in config:
        for motion in config['motions']:
            if isinstance(motion, dict) and 'file' in motion:
                # 元のファイル名を維持しつつ、ディレクトリを変更
                basename = os.path.basename(motion['file'])
                motion['file'] = os.path.join(output_motion_dir, basename)
    
    # メタデータを更新
    config['num_dof'] = 23
    config['description'] = '23DOF G1 motion dataset (converted from 29DOF)'
    
    with open(output_yaml, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f"Created 23DOF dataset YAML: {output_yaml}")


def main():
    parser = argparse.ArgumentParser(description="Convert 29DOF motion data to 23DOF")
    parser.add_argument('--input_dir', type=str, help='Input directory with 29DOF motion files')
    parser.add_argument('--output_dir', type=str, required=True, help='Output directory for 23DOF motion files')
    parser.add_argument('--yaml', type=str, help='Input dataset YAML file')
    parser.add_argument('--output_yaml', type=str, help='Output 23DOF dataset YAML file')
    
    args = parser.parse_args()
    
    if args.yaml:
        success, failed = convert_from_yaml(args.yaml, args.output_dir)
        
        if args.output_yaml:
            create_23dof_yaml(args.yaml, args.output_yaml, args.output_dir)
    elif args.input_dir:
        success, failed = convert_directory(args.input_dir, args.output_dir)
    else:
        parser.error("Either --input_dir or --yaml must be specified")
        return
    
    print(f"\nConversion complete: {success} succeeded, {failed} failed")
    print(f"Output directory: {args.output_dir}")


if __name__ == '__main__':
    main()

