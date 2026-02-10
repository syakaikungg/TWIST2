#!/usr/bin/env python3
"""
Play back recorded motion from data.json
Reads JSON data and publishes to Redis for sim2sim/sim2real
"""
import argparse
import time
import redis
import json
import numpy as np


def main(args):
    # Connect to Redis
    redis_client = redis.Redis(host=args.redis_ip, port=6379, db=0)
    redis_client.ping()
    print(f"✅ Connected to Redis at {args.redis_ip}")
    
    # Load recorded data
    print(f"\nLoading recorded motion from: {args.data_file}")
    with open(args.data_file, 'r') as f:
        json_data = json.load(f)
    
    # Extract the data array
    if 'data' in json_data:
        data_frames = json_data['data']
    else:
        data_frames = json_data
    
    num_frames = len(data_frames)
    print(f"✅ Loaded {num_frames} frames")
    
    # Extract mimic_obs from each frame
    mimic_obs_list = []
    for frame in data_frames:
        mimic_obs = frame.get('action_body', None)
        if mimic_obs is not None:
            mimic_obs_list.append(mimic_obs)
    
    if len(mimic_obs_list) == 0:
        print("❌ No mimic_obs data found in recording!")
        return
    
    print(f"✅ Extracted {len(mimic_obs_list)} mimic_obs frames")
    print(f"   Mimic obs dimension: {len(mimic_obs_list[0])}")
    print(f"   Duration: {len(mimic_obs_list) / args.playback_frequency:.2f} seconds")
    print(f"   Playback frequency: {args.playback_frequency} Hz\n")
    
    control_dt = 1.0 / args.playback_frequency
    
    print("━" * 70)
    print("Starting motion playback (looping)")
    print("Press Ctrl+C to stop")
    print("━" * 70)
    print()
    
    frame_idx = 0
    try:
        while True:
            loop_start = time.time()
            
            # Get current frame
            mimic_obs = mimic_obs_list[frame_idx]
            
            # Publish to Redis
            redis_client.set("action_body_unitree_g1_with_hands", json.dumps(mimic_obs))
            
            # Also publish hand and neck data (zeros for now)
            redis_client.set("action_hand_left_unitree_g1_with_hands", json.dumps(np.zeros(6).tolist()))
            redis_client.set("action_hand_right_unitree_g1_with_hands", json.dumps(np.zeros(6).tolist()))
            redis_client.set("action_neck_unitree_g1_with_hands", json.dumps([0.0, 0.0]))
            
            # Print status every second
            if frame_idx % args.playback_frequency == 0:
                elapsed = frame_idx * control_dt
                print(f"[{elapsed:6.1f}s] Frame {frame_idx}/{len(mimic_obs_list)} (Loop: {frame_idx // len(mimic_obs_list) + 1})")
            
            # Move to next frame
            frame_idx += 1
            if frame_idx >= len(mimic_obs_list):
                frame_idx = 0  # Loop back to start
                print(f"\n{'='*70}")
                print("Motion finished. Restarting...")
                print(f"{'='*70}\n")
            
            # Maintain playback rate
            elapsed = time.time() - loop_start
            if elapsed < control_dt:
                time.sleep(control_dt - elapsed)
                
    except KeyboardInterrupt:
        print("\n\nStopping playback...")
    finally:
        # Send default pose
        print("Sending default pose to Redis...")
        default_mimic_obs = [0.0, 0.0, 0.793, 0.0, 0.0, 0.0] + [0.0] * 29
        redis_client.set("action_body_unitree_g1_with_hands", json.dumps(default_mimic_obs))
        print("Playback stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Play back recorded motion from JSON')
    parser.add_argument('--data_file', type=str, required=True,
                        help='Path to data.json file')
    parser.add_argument('--redis_ip', type=str, default='localhost',
                        help='Redis server IP')
    parser.add_argument('--playback_frequency', type=int, default=30,
                        help='Playback frequency in Hz (default: 30)')
    
    args = parser.parse_args()
    main(args)
