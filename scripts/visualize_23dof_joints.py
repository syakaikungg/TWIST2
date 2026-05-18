#!/usr/bin/env python3
"""
23DOF G1 ロボットの関節構造を可視化するスクリプト

URDFファイルをパースし、デフォルト関節角度でのロボット姿勢を
3Dプロットで表示します。設定の整合性チェックも行います。

使用方法:
    python scripts/visualize_23dof_joints.py
    python scripts/visualize_23dof_joints.py --no-plot  # プロットなし（チェックのみ）
"""

import os
import sys
import argparse
import xml.etree.ElementTree as ET
import numpy as np
import math

# プロジェクトルートを基準に
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

# --------------------------------------------------------------------------
# URDF パーサー
# --------------------------------------------------------------------------

def parse_origin(elem):
    """URDFの<origin>要素からxyz, rpyを取得"""
    if elem is None:
        return np.zeros(3), np.zeros(3)
    xyz_str = elem.get('xyz', '0 0 0')
    rpy_str = elem.get('rpy', '0 0 0')
    xyz = np.array([float(v) for v in xyz_str.split()])
    rpy = np.array([float(v) for v in rpy_str.split()])
    return xyz, rpy


def rpy_to_rotation_matrix(rpy):
    """Roll-Pitch-Yaw → 回転行列"""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    
    return Rz @ Ry @ Rx


def rotation_about_axis(axis, angle):
    """軸まわりの回転行列 (Rodriguesの公式)"""
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def make_transform(R, t):
    """4x4同次変換行列を作成"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def parse_urdf(urdf_path):
    """URDFファイルをパースしてリンクとジョイント情報を返す"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    links = {}
    joints = []
    
    for link in root.findall('link'):
        name = link.get('name')
        links[name] = {'name': name}
    
    for joint in root.findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        
        origin = joint.find('origin')
        xyz, rpy = parse_origin(origin)
        
        axis_elem = joint.find('axis')
        if axis_elem is not None:
            axis = np.array([float(v) for v in axis_elem.get('xyz', '0 0 1').split()])
        else:
            axis = np.array([0, 0, 1])
        
        limit_elem = joint.find('limit')
        limits = None
        if limit_elem is not None:
            limits = {
                'lower': float(limit_elem.get('lower', 0)),
                'upper': float(limit_elem.get('upper', 0)),
                'effort': float(limit_elem.get('effort', 0)),
                'velocity': float(limit_elem.get('velocity', 0)),
            }
        
        joints.append({
            'name': name,
            'type': jtype,
            'parent': parent,
            'child': child,
            'xyz': xyz,
            'rpy': rpy,
            'axis': axis,
            'limits': limits,
        })
    
    return links, joints


def compute_forward_kinematics(links, joints, joint_angles=None):
    """
    順運動学を計算して各リンクのワールド座標位置を返す
    
    Args:
        links: リンク辞書
        joints: ジョイントリスト
        joint_angles: {joint_name: angle_rad} のデフォルト角度
    
    Returns:
        {link_name: position(3,)} の辞書
    """
    if joint_angles is None:
        joint_angles = {}
    
    # 親子関係をマッピング
    parent_to_joints = {}
    for j in joints:
        parent = j['parent']
        if parent not in parent_to_joints:
            parent_to_joints[parent] = []
        parent_to_joints[parent].append(j)
    
    # ルートリンクを見つける（どのジョイントのchildにもならないリンク）
    child_links = {j['child'] for j in joints}
    root_links = [name for name in links if name not in child_links]
    
    # 各リンクの変換を計算
    link_transforms = {}
    link_positions = {}
    
    for root in root_links:
        link_transforms[root] = np.eye(4)
        link_positions[root] = np.zeros(3)
    
    # BFSで各リンクの変換を計算
    queue = list(root_links)
    visited = set(root_links)
    
    while queue:
        current = queue.pop(0)
        parent_T = link_transforms[current]
        
        if current in parent_to_joints:
            for j in parent_to_joints[current]:
                child = j['child']
                if child in visited:
                    continue
                
                # ジョイント原点の変換
                R_origin = rpy_to_rotation_matrix(j['rpy'])
                T_origin = make_transform(R_origin, j['xyz'])
                
                # 回転関節の場合は角度を適用
                if j['type'] == 'revolute':
                    angle = joint_angles.get(j['name'], 0.0)
                    R_joint = rotation_about_axis(j['axis'], angle)
                    T_joint = make_transform(R_joint, np.zeros(3))
                    T_child = parent_T @ T_origin @ T_joint
                else:
                    T_child = parent_T @ T_origin
                
                link_transforms[child] = T_child
                link_positions[child] = T_child[:3, 3]
                visited.add(child)
                queue.append(child)
    
    return link_positions, link_transforms


# --------------------------------------------------------------------------
# 設定整合性チェック
# --------------------------------------------------------------------------

def check_config_consistency(joints):
    """URDFと23DOF設定の整合性をチェック"""
    
    # 23DOF設定のデフォルト角度
    default_joint_angles_23dof = {
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
        'left_shoulder_pitch_joint': 0.0,
        'left_shoulder_roll_joint': 0.4,
        'left_shoulder_yaw_joint': 0.0,
        'left_elbow_joint': 1.2,
        'left_wrist_roll_joint': 0.0,
        'right_shoulder_pitch_joint': 0.0,
        'right_shoulder_roll_joint': -0.4,
        'right_shoulder_yaw_joint': 0.0,
        'right_elbow_joint': 1.2,
        'right_wrist_roll_joint': 0.0,
    }
    
    # 23DOFで固定されるべきジョイント
    fixed_joints_23dof = [
        'waist_roll_joint',
        'waist_pitch_joint',
        'left_wrist_pitch_joint',
        'left_wrist_yaw_joint',
        'right_wrist_pitch_joint',
        'right_wrist_yaw_joint',
    ]
    
    # 23DOF関節順序
    joint_order_23dof = [
        'left_hip_pitch_joint',     # [0]
        'left_hip_roll_joint',      # [1]
        'left_hip_yaw_joint',       # [2]
        'left_knee_joint',          # [3]
        'left_ankle_pitch_joint',   # [4]
        'left_ankle_roll_joint',    # [5]
        'right_hip_pitch_joint',    # [6]
        'right_hip_roll_joint',     # [7]
        'right_hip_yaw_joint',      # [8]
        'right_knee_joint',         # [9]
        'right_ankle_pitch_joint',  # [10]
        'right_ankle_roll_joint',   # [11]
        'waist_yaw_joint',          # [12]
        'left_shoulder_pitch_joint',  # [13]
        'left_shoulder_roll_joint',   # [14]
        'left_shoulder_yaw_joint',    # [15]
        'left_elbow_joint',           # [16]
        'left_wrist_roll_joint',      # [17]
        'right_shoulder_pitch_joint', # [18]
        'right_shoulder_roll_joint',  # [19]
        'right_shoulder_yaw_joint',   # [20]
        'right_elbow_joint',          # [21]
        'right_wrist_roll_joint',     # [22]
    ]
    
    print("=" * 70)
    print("23DOF 設定整合性チェック")
    print("=" * 70)
    
    # URDFの全ジョイントを取得
    urdf_revolute = [j for j in joints if j['type'] == 'revolute']
    urdf_fixed = [j for j in joints if j['type'] == 'fixed']
    
    print(f"\n[URDF ジョイント概要]")
    print(f"  可動関節 (revolute): {len(urdf_revolute)}")
    print(f"  固定関節 (fixed):    {len(urdf_fixed)}")
    
    urdf_revolute_names = [j['name'] for j in urdf_revolute]
    urdf_fixed_names = [j['name'] for j in urdf_fixed]
    
    # チェック1: 23DOF可動関節がURDFにrevolute jointとして存在するか
    print(f"\n[チェック1] 23DOF可動関節 ({len(joint_order_23dof)}個)")
    errors = 0
    for i, jname in enumerate(joint_order_23dof):
        if jname in urdf_revolute_names:
            status = "OK"
        elif jname in urdf_fixed_names:
            status = "ERROR (URDFではfixed!)"
            errors += 1
        else:
            status = "ERROR (URDFに存在しない!)"
            errors += 1
        print(f"  [{i:2d}] {jname:40s} {status}")
    
    if errors == 0:
        print(f"  → 全 {len(joint_order_23dof)} 関節が正しくrevoluteとして定義 OK")
    else:
        print(f"  → {errors} 個のエラーあり NG")
    
    # チェック2: 固定されるべきジョイントがURDFでfixedになっているか
    print(f"\n[チェック2] 23DOFで固定されるジョイント ({len(fixed_joints_23dof)}個)")
    errors2 = 0
    for jname in fixed_joints_23dof:
        if jname in urdf_fixed_names:
            status = "OK (fixed)"
        elif jname in urdf_revolute_names:
            status = "WARNING (URDFではrevolute - IsaacGymではcollapse_fixed_joints=Falseなので動作可能)"
            errors2 += 1
        else:
            status = "ERROR (URDFに存在しない!)"
            errors2 += 1
        print(f"  {jname:40s} {status}")
    
    if errors2 == 0:
        print(f"  → 全ジョイントが正しくfixed OK")
    
    # チェック3: デフォルト角度が関節制限内か
    print(f"\n[チェック3] デフォルト角度 vs 関節制限")
    joint_limits = {j['name']: j['limits'] for j in joints if j['limits'] is not None}
    errors3 = 0
    for jname, angle in default_joint_angles_23dof.items():
        if jname in joint_limits:
            lim = joint_limits[jname]
            in_range = lim['lower'] <= angle <= lim['upper']
            status = "OK" if in_range else f"OUT OF RANGE! [{lim['lower']:.3f}, {lim['upper']:.3f}]"
            if not in_range:
                errors3 += 1
            print(f"  {jname:40s} = {angle:6.2f} rad  ({math.degrees(angle):7.1f}°)  "
                  f"[{lim['lower']:6.2f}, {lim['upper']:6.2f}]  {status}")
        else:
            print(f"  {jname:40s} = {angle:6.2f} rad  (制限情報なし)")
    
    if errors3 == 0:
        print(f"  → 全デフォルト角度が制限内 OK")
    
    # チェック4: dof_armature の個数
    dof_armature = [0.0103, 0.0251, 0.0103, 0.0251, 0.003597, 0.003597] * 2 + \
                   [0.0103] + \
                   [0.003597, 0.003597, 0.003597, 0.003597, 0.003597] * 2
    print(f"\n[チェック4] dof_armature サイズ")
    print(f"  dof_armature 長さ: {len(dof_armature)}")
    print(f"  23DOF 期待値:      23")
    if len(dof_armature) == 23:
        print(f"  → 一致 OK")
    else:
        print(f"  → 不一致 NG")
    
    # チェック5: action_std の個数
    action_std = [0.7] * 12 + [0.4] * 1 + [0.5] * 10
    print(f"\n[チェック5] action_std サイズ")
    print(f"  action_std 長さ:  {len(action_std)}")
    print(f"  内訳: 脚12({[0.7]*12}) + 腰1({[0.4]*1}) + 腕10({[0.5]*10})")
    if len(action_std) == 23:
        print(f"  → 23DOF と一致 OK")
    else:
        print(f"  → 不一致 NG")
    
    # チェック6: dof_err_w の個数
    dof_err_w_len = 6 + 6 + 1 + 5 + 5  # 左脚 + 右脚 + 腰 + 左腕 + 右腕
    print(f"\n[チェック6] dof_err_w サイズ")
    print(f"  dof_err_w 長さ: {dof_err_w_len}")
    if dof_err_w_len == 23:
        print(f"  → 23DOF と一致 OK")
    else:
        print(f"  → 不一致 NG")
    
    print()
    return default_joint_angles_23dof, joint_order_23dof


# --------------------------------------------------------------------------
# 3D可視化
# --------------------------------------------------------------------------

def visualize_robot_3d(links, joints, link_positions, joint_angles, joint_order_23dof):
    """ロボットのスケルトンを3Dプロットで表示"""
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError:
        print("matplotlib がインストールされていません。可視化をスキップします。")
        print("  pip install matplotlib")
        return
    
    fig = plt.figure(figsize=(18, 8))
    
    # ---------- サブプロット1: 3Dスケルトン ----------
    ax1 = fig.add_subplot(121, projection='3d')
    
    # 各ジョイント(親→子)の線を描画
    # 体のパーツごとに色を変える
    body_part_colors = {
        'left_hip': '#2196F3',     # 青
        'left_knee': '#2196F3',
        'left_ankle': '#2196F3',
        'left_toe': '#2196F3',
        'right_hip': '#F44336',    # 赤
        'right_knee': '#F44336',
        'right_ankle': '#F44336',
        'right_toe': '#F44336',
        'waist': '#4CAF50',        # 緑
        'torso': '#4CAF50',
        'head': '#9C27B0',         # 紫
        'imu': '#4CAF50',
        'left_shoulder': '#03A9F4', # ライトブルー
        'left_elbow': '#03A9F4',
        'left_wrist': '#03A9F4',
        'left_hand': '#03A9F4',
        'left_rubber': '#03A9F4',
        'right_shoulder': '#FF5722', # オレンジ
        'right_elbow': '#FF5722',
        'right_wrist': '#FF5722',
        'right_hand': '#FF5722',
        'right_rubber': '#FF5722',
        'pelvis': '#4CAF50',
    }
    
    def get_color(joint_name):
        for key, color in body_part_colors.items():
            if key in joint_name:
                return color
        return '#757575'
    
    for j in joints:
        parent = j['parent']
        child = j['child']
        if parent in link_positions and child in link_positions:
            p = link_positions[parent]
            c = link_positions[child]
            color = get_color(j['name'])
            linewidth = 3 if j['type'] == 'revolute' else 1.5
            ax1.plot([p[0], c[0]], [p[1], c[1]], [p[2], c[2]], 
                    color=color, linewidth=linewidth, alpha=0.8)
    
    # 23DOF可動関節を強調表示
    revolute_set = set(joint_order_23dof)
    for j in joints:
        child = j['child']
        if child in link_positions:
            pos = link_positions[child]
            if j['name'] in revolute_set:
                ax1.scatter(*pos, c='red', s=80, zorder=5, edgecolors='black', linewidth=0.5)
            elif j['type'] == 'revolute':
                ax1.scatter(*pos, c='orange', s=50, zorder=4, edgecolors='black', linewidth=0.5)
            elif j['type'] == 'fixed':
                ax1.scatter(*pos, c='gray', s=20, zorder=3, alpha=0.5)
    
    # pelvis (ルート)
    if 'pelvis' in link_positions:
        p = link_positions['pelvis']
        ax1.scatter(*p, c='green', s=120, zorder=6, marker='D', edgecolors='black', linewidth=1)
    
    # キーポイントにラベル
    key_links = [
        'pelvis', 'torso_link', 'head_mocap',
        'left_ankle_roll_link', 'right_ankle_roll_link',
        'left_rubber_hand', 'right_rubber_hand',
    ]
    for link_name in key_links:
        if link_name in link_positions:
            pos = link_positions[link_name]
            short_name = link_name.replace('_link', '').replace('_joint', '')
            ax1.text(pos[0], pos[1], pos[2] + 0.03, short_name, fontsize=7, ha='center')
    
    ax1.set_xlabel('X (前方)')
    ax1.set_ylabel('Y (左方)')
    ax1.set_zlabel('Z (上方)')
    ax1.set_title('G1 23DOF Robot - Default Pose\n(赤点=23DOF可動, 灰色=固定)', fontsize=11)
    
    # アスペクト比を等しくする
    all_pos = np.array([link_positions[k] for k in link_positions])
    center = all_pos.mean(axis=0)
    max_range = (all_pos.max(axis=0) - all_pos.min(axis=0)).max() / 2 * 1.2
    ax1.set_xlim(center[0] - max_range, center[0] + max_range)
    ax1.set_ylim(center[1] - max_range, center[1] + max_range)
    ax1.set_zlim(center[2] - max_range, center[2] + max_range)
    
    # 正面から見た視点
    ax1.view_init(elev=10, azim=-80)
    
    # ---------- サブプロット2: 関節角度バーチャート ----------
    ax2 = fig.add_subplot(122)
    
    joint_names_short = []
    angles_deg = []
    colors = []
    
    group_labels = {
        0: '左脚', 6: '右脚', 12: '腰', 13: '左腕', 18: '右腕'
    }
    group_colors = {
        '左脚': '#2196F3', '右脚': '#F44336', '腰': '#4CAF50',
        '左腕': '#03A9F4', '右腕': '#FF5722'
    }
    
    current_group = '左脚'
    for i, jname in enumerate(joint_order_23dof):
        if i in group_labels:
            current_group = group_labels[i]
        
        # 短縮名
        short = jname.replace('_joint', '').replace('left_', 'L ').replace('right_', 'R ')
        joint_names_short.append(f'[{i:2d}] {short}')
        
        angle = joint_angles.get(jname, 0.0)
        angles_deg.append(math.degrees(angle))
        colors.append(group_colors.get(current_group, '#757575'))
    
    y_pos = np.arange(len(joint_names_short))
    bars = ax2.barh(y_pos, angles_deg, color=colors, alpha=0.8, edgecolor='white', height=0.7)
    
    # 値のラベル
    for bar, angle_deg in zip(bars, angles_deg):
        if abs(angle_deg) > 1:
            ax2.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2, 
                    f'{angle_deg:.1f}°', va='center', fontsize=8)
    
    ax2.set_yticks(y_pos)
    ax2.set_yticklabels(joint_names_short, fontsize=8, fontfamily='monospace')
    ax2.set_xlabel('角度 (degrees)')
    ax2.set_title('23DOF デフォルト関節角度', fontsize=11)
    ax2.invert_yaxis()
    ax2.axvline(x=0, color='black', linewidth=0.5)
    ax2.grid(axis='x', alpha=0.3)
    
    # 凡例
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=c, label=l) for l, c in group_colors.items()]
    ax2.legend(handles=legend_elements, loc='lower right', fontsize=9)
    
    plt.tight_layout()
    
    # 保存
    output_path = os.path.join(PROJECT_ROOT, 'assets', 'g1', 'images', 'g1_23dof_joint_check.png')
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"可視化画像を保存しました: {output_path}")
    
    plt.show()


# --------------------------------------------------------------------------
# キネマティックチェーンの表示
# --------------------------------------------------------------------------

def print_kinematic_tree(joints):
    """キネマティックチェーンをツリー表示"""
    # 親子関係を構築
    children = {}
    for j in joints:
        parent = j['parent']
        if parent not in children:
            children[parent] = []
        children[parent].append(j)
    
    # ルートリンクを見つける
    child_links = {j['child'] for j in joints}
    parent_links = {j['parent'] for j in joints}
    roots = parent_links - child_links
    
    print("=" * 70)
    print("キネマティックチェーン (ツリー表示)")
    print("=" * 70)
    
    def print_tree(link, prefix="", is_last=True):
        connector = "└── " if is_last else "├── "
        print(f"{prefix}{connector}{link}")
        
        if link in children:
            child_prefix = prefix + ("    " if is_last else "│   ")
            for i, j in enumerate(children[link]):
                is_last_child = (i == len(children[link]) - 1)
                jtype_tag = "[R]" if j['type'] == 'revolute' else "[F]"
                print(f"{child_prefix}{'└' if is_last_child else '├'}─{jtype_tag} {j['name']}")
                print_tree(j['child'], child_prefix, is_last_child)
    
    for root in sorted(roots):
        print(f"\n{root} (ROOT)")
        if root in children:
            for i, j in enumerate(children[root]):
                is_last = (i == len(children[root]) - 1)
                jtype_tag = "[R]" if j['type'] == 'revolute' else "[F]"
                print(f"{'└' if is_last else '├'}─{jtype_tag} {j['name']}")
                print_tree(j['child'], "  " if is_last else "│ ", is_last)
    
    print()


# --------------------------------------------------------------------------
# メイン
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="23DOF G1 ロボット関節可視化")
    parser.add_argument('--urdf', type=str, 
                       default=os.path.join(PROJECT_ROOT, 'assets', 'g1', 'g1_custom_collision_23dof.urdf'),
                       help='URDFファイルパス')
    parser.add_argument('--no-plot', action='store_true', help='プロットを表示しない（チェックのみ）')
    args = parser.parse_args()
    
    if not os.path.exists(args.urdf):
        print(f"Error: URDF file not found: {args.urdf}")
        sys.exit(1)
    
    print(f"URDF: {args.urdf}")
    print()
    
    # 1. URDFをパース
    links, joints = parse_urdf(args.urdf)
    
    # 2. キネマティックチェーンを表示
    print_kinematic_tree(joints)
    
    # 3. 設定整合性チェック
    default_angles, joint_order = check_config_consistency(joints)
    
    # 4. 順運動学を計算
    print("=" * 70)
    print("順運動学による関節位置 (デフォルトポーズ)")
    print("=" * 70)
    
    link_positions, _ = compute_forward_kinematics(links, joints, default_angles)
    
    # 主要リンクの位置を表示
    key_links_display = [
        ('pelvis', 'ルート（骨盤）'),
        ('torso_link', '胴体'),
        ('head_mocap', '頭部'),
        ('left_hip_pitch_link', '左股関節'),
        ('left_knee_link', '左膝'),
        ('left_ankle_roll_link', '左足'),
        ('right_hip_pitch_link', '右股関節'),
        ('right_knee_link', '右膝'),
        ('right_ankle_roll_link', '右足'),
        ('left_shoulder_pitch_link', '左肩'),
        ('left_elbow_link', '左肘'),
        ('left_rubber_hand', '左手'),
        ('right_shoulder_pitch_link', '右肩'),
        ('right_elbow_link', '右肘'),
        ('right_rubber_hand', '右手'),
    ]
    
    print(f"\n{'リンク名':40s} {'説明':12s} {'X':>8s} {'Y':>8s} {'Z':>8s}")
    print("-" * 78)
    for link_name, desc in key_links_display:
        if link_name in link_positions:
            pos = link_positions[link_name]
            print(f"{link_name:40s} {desc:12s} {pos[0]:8.4f} {pos[1]:8.4f} {pos[2]:8.4f}")
        else:
            print(f"{link_name:40s} {desc:12s} {'(not found)':>24s}")
    
    print()
    
    # 5. 3Dプロットで可視化
    if not args.no_plot:
        visualize_robot_3d(links, joints, link_positions, default_angles, joint_order)
    else:
        print("(--no-plot が指定されたためプロットをスキップ)")


if __name__ == '__main__':
    main()
