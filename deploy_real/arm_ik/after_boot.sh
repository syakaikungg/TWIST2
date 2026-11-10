#!/bin/bash
# Run once after the G1 has been (re)booted. From the operator PC: bash g1.sh up
# It gets the G1 ready for teleop; nothing here moves the robot (the arms start separately with run_live.sh).
#   1) Redis + PC Service (Docker)        nextech-redis, xrobot-service
#   2) head camera USB power saving off   power/control = on for the camera (2-2.3, only if it is the Intel
#                                         camera) and its hub (2-2)
#   3) head camera -> PICO for teleop     cam_to_pico.sh boot (deploy_real/vr_camera): Unitree's video_hub_pc4
#                                         off (it comes back on every reboot and holds the camera), then the
#                                         real-camera sender at 960x540 waiting for the PICO on CAM_LISTEN
#                                         (default 0.0.0.0:13579 since 2026-11-10 = wired and Wi-Fi; it was
#                                         192.168.123.164:13579, the wired side only)
# Password: after the one-time install_autostart.sh (2026-11-10), Docker starts 1) by itself at boot and a udev rule
# does 2), so this script uses no sudo and asks for no password (Claude can run it over SSH). Without that, or if
# they are not up within UP_WAIT s (default 40) after a boot, it falls back to sudo as before (password once, only
# from a terminal; without a terminal it stops and says so).
# Then on the PICO: XRobot app -> Remote Vision -> ZEDMINI -> Listen -> the G1's address (printed at the end) -> Confirm.
# It stops before doing anything while the arm programs run: restarting the camera would blank the operator's view.
set -u
CAM_USB=2-2.3
HUB_USB=2-2
CAM=~/nextech/cam_to_pico/cam_to_pico.sh
CAM_LISTEN=${CAM_LISTEN:-0.0.0.0:13579}
UP_WAIT=${UP_WAIT:-40}
RULE=/etc/udev/rules.d/90-nextech-head-camera-power.rules   # written by install_autostart.sh
step() { echo; echo "== $*"; }
listening() { ss -ltn 2>/dev/null | grep -qE ":$1\b"; }
# sudo: with a terminal it may ask for the password; without one it must not (no prompt can be answered)
TTY=0; [ -t 0 ] && TTY=1
su_do() {
  if [ "$TTY" = 1 ] || sudo -n true 2> /dev/null; then
    sudo "$@"
  else
    echo "needs the sudo password: run 'bash g1.sh up' in a terminal, or once 'bash g1.sh autostart'" >&2
    return 1
  fi
}
[[ "$CAM_LISTEN" =~ ^[0-9.]+:[0-9]+$ ]] || { echo "CAM_LISTEN must be address:port (e.g. 0.0.0.0:13579)"; exit 1; }
[[ "$UP_WAIT" =~ ^[0-9]+$ ]] || { echo "UP_WAIT must be a whole number of seconds"; exit 1; }

if ps -eo cmd | grep -E 'arm_sdk_bridge|arm_ik_dryrun' | grep -v grep > /dev/null; then
  echo "ABORT: the arm programs are running (stop them first)"
  exit 1
fi

step "1/3 Redis and PC Service"
if [ -e "$RULE" ] && ! { listening 6379 && listening 63901; }; then
  # set up by install_autostart.sh: Docker starts them by itself, which can take a while just after a boot
  echo "waiting up to $UP_WAIT s for Docker to start them (restart=unless-stopped)"
  for ((i = 0; i < UP_WAIT; i++)); do
    listening 6379 && listening 63901 && break
    sleep 1
  done
fi
if listening 6379 && listening 63901; then
  echo "already running (Redis 6379, PC Service 63901): no sudo needed"
else
  su_do docker start nextech-redis xrobot-service || { echo "docker start failed"; exit 1; }
fi

step "2/3 head camera USB power saving off"
if [ "$(cat /sys/bus/usb/devices/$CAM_USB/idVendor 2>/dev/null)" = "8086" ]; then
  for d in "$CAM_USB" "$HUB_USB"; do
    f=/sys/bus/usb/devices/$d/power/control
    if [ -e "$f" ]; then
      if [ "$(cat "$f")" != on ]; then
        echo on | su_do tee "$f" > /dev/null || echo "$d: power saving left as it is"
      fi
      echo "$d ($(cat /sys/bus/usb/devices/$d/product 2>/dev/null)): power/control=$(cat "$f")"
    fi
  done
else
  echo "the camera is not at USB $CAM_USB: power saving left as it is"
fi

step "3/3 head camera -> PICO (cam_to_pico.sh boot)"
if grep -qE '^ *boot\)' "$CAM" 2>/dev/null; then
  bash "$CAM" boot --listen "$CAM_LISTEN"
else
  echo "camera NOT started: $CAM is missing or has no 'boot' (put the current vr_camera files on the G1)"
fi

step "status"
sleep 1
listening 6379 && echo "Redis 6379: listening" || echo "Redis 6379: NOT listening"
listening 63901 && echo "PC Service 63901: listening" || echo "PC Service 63901: NOT listening"
listening "${CAM_LISTEN##*:}" && echo "camera ${CAM_LISTEN##*:}: waiting for the PICO ($CAM_LISTEN)" || echo "camera ${CAM_LISTEN##*:}: NOT listening"
wifi=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n 1)
echo
echo "PICO: XRobot app -> PC Service and Remote Vision (ZEDMINI) -> Listen -> 192.168.123.164 (wired)${wifi:+ or $wifi (Wi-Fi)}"
echo "arms: bash g1.sh dry|send on the operator PC (= run_live.sh), only after an explicit go"
