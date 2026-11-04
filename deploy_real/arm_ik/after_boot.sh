#!/bin/bash
# Run once after the G1 has been (re)booted. From the laptop's PowerShell (asks for the sudo password once):
#   wsl -d Ubuntu -- ssh -t -i /home/masak/.ssh/id_ed25519_g1 unitree@192.168.123.164 "bash ~/nextech/arm_ik/after_boot.sh"
# It gets the G1 ready for teleop; nothing here moves the robot (the arms start separately with run_live.sh).
#   1) Redis + PC Service (Docker)        sudo docker start nextech-redis xrobot-service
#   2) head camera USB power saving off   power/control = on for the camera (2-2.3, only if it is the Intel
#                                         camera) and its hub (2-2); this resets on every reboot
#   3) head camera -> PICO for teleop     cam_to_pico.sh boot (deploy_real/vr_camera): Unitree's video_hub_pc4
#                                         off (it comes back on every reboot and holds the camera), then the
#                                         real-camera sender at 960x540 waiting for the PICO
# Then on the PICO: XRobot app -> Remote Vision -> ZEDMINI -> Listen -> 192.168.123.164 -> Confirm.
# It stops before doing anything while the arm programs run: restarting the camera would blank the operator's view.
set -u
CAM_USB=2-2.3
HUB_USB=2-2
CAM=~/nextech/cam_to_pico/cam_to_pico.sh
step() { echo; echo "== $*"; }

if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun' | grep -v grep > /dev/null; then
  echo "ABORT: the arm programs are running (stop them first)"
  exit 1
fi

step "1/3 Redis and PC Service"
sudo docker start nextech-redis xrobot-service || { echo "docker start failed"; exit 1; }

step "2/3 head camera USB power saving off"
if [ "$(cat /sys/bus/usb/devices/$CAM_USB/idVendor 2>/dev/null)" = "8086" ]; then
  for d in "$CAM_USB" "$HUB_USB"; do
    f=/sys/bus/usb/devices/$d/power/control
    if [ -e "$f" ]; then
      echo on | sudo tee "$f" > /dev/null
      echo "$d ($(cat /sys/bus/usb/devices/$d/product 2>/dev/null)): power/control=$(cat "$f")"
    fi
  done
else
  echo "the camera is not at USB $CAM_USB: power saving left as it is"
fi

step "3/3 head camera -> PICO (cam_to_pico.sh boot)"
if grep -qE '^ *boot\)' "$CAM" 2>/dev/null; then
  bash "$CAM" boot
else
  echo "camera NOT started: $CAM is missing or has no 'boot' (put the current vr_camera files on the G1)"
fi

step "status"
docker_state=$(sudo docker ps --format '{{.Names}} {{.Status}}' | grep -E 'nextech-redis|xrobot-service')
echo "${docker_state:-docker: nothing running}"
ss -ltn 2>/dev/null | grep -qE ':6379\b' && echo "Redis 6379: listening" || echo "Redis 6379: NOT listening"
ss -ltn 2>/dev/null | grep -qE ':63901\b' && echo "PC Service 63901: listening" || echo "PC Service 63901: NOT listening"
ss -ltn 2>/dev/null | grep -qE ':13579\b' && echo "camera 13579: waiting for the PICO" || echo "camera 13579: NOT listening"
echo
echo "PICO: XRobot app -> Remote Vision -> ZEDMINI -> Listen -> 192.168.123.164 -> Confirm"
echo "arms: bash ~/nextech/arm_ik/run_live.sh dry|send (from the laptop, after an explicit go)"
