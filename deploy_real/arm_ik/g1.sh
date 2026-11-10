#!/bin/bash
# Run on the operator PC (Ubuntu, or WSL Ubuntu on the Windows laptop), NOT on the G1. The teleop itself runs on the
# G1's onboard PC; this only starts and stops it over SSH.
#   g1.sh up                                after a G1 (re)boot: Redis + PC Service, head camera USB power saving off,
#                                           head camera -> PICO on 0.0.0.0:13579 (after_boot.sh). Moves nothing.
#                                           No password once `g1.sh autostart` has been done; before that it asks for
#                                           the G1 sudo password once (only in a terminal).
#   g1.sh autostart [--check|--undo]        ONCE: install_autostart.sh (sudo password once): Redis + PC Service start
#                                           with the G1 and a udev rule turns the camera USB power saving off.
#   g1.sh dry  [max_vel] [seconds] [tool]   run_live.sh dry: IK + bridge, the bridge only logs (nothing is sent)
#   g1.sh send [max_vel] [seconds] [tool]   run_live.sh send: the arms MOVE. Only after an explicit go, with the
#                                           robot hanging on its rope while testing. Defaults 0.5 rad/s, 180 s, hand.
#   g1.sh stop                              stop.sh: the arms go back to the robot's own controller in 3 s (= left X)
#   g1.sh status                            what runs on the G1 (read only): teleop state, ports, camera
#   g1.sh ssh                               a shell on the G1
# Settings (environment): G1_HOST (default unitree@192.168.123.164), G1_KEY (default ~/.ssh/id_ed25519_g1),
# MAX_TEMP (passed to run_live.sh: 50..70 C, default 70; none = no temperature stop, one measurement run only),
# WAIST_CORRECT (passed to run_live.sh: empty = the bridge's integral waist correction off, the default since 2026-11-10; roll,pitch = on as before), CAM_LISTEN (passed to
# after_boot.sh: address:port the camera waits on, default 0.0.0.0:13579), TEMP_PAUSE / TEMP_RESUME (passed to
# run_live.sh: at TEMP_PAUSE C the arms glide back to the start pose and pause until TEMP_RESUME C; default off (0) / 60).
# Setting up a new PC (wired IP, SSH key): README.md, "操作用のPC".
set -u
HOST=${G1_HOST:-unitree@192.168.123.164}
KEY=${G1_KEY:-$HOME/.ssh/id_ed25519_g1}
SSH=(ssh -i "$KEY" -o ConnectTimeout=8 "$HOST")
if [ ! -r "$KEY" ]; then
  echo "SSH key $KEY not found. Make one and put its public key on the G1 (README.md, 操作用のPC):"
  echo "  ssh-keygen -t ed25519 -f $KEY && ssh-copy-id -i $KEY.pub $HOST"
  exit 1
fi
usage() { sed -n 4,22p "$0"; exit 1; }
TTY=(); [ -t 0 ] && TTY=(-t)    # a terminal only when there is one (Claude runs this without one)

cmd=${1:-}
[ $# -gt 0 ] && shift
case "$cmd" in
  up)
    cl=""
    if [ -n "${CAM_LISTEN:-}" ]; then
      [[ "$CAM_LISTEN" =~ ^[0-9.]+:[0-9]+$ ]] || { echo "CAM_LISTEN must be address:port (e.g. 0.0.0.0:13579)"; exit 1; }
      cl="CAM_LISTEN=$CAM_LISTEN "
    fi
    exec ssh "${TTY[@]}" -i "$KEY" -o ConnectTimeout=8 "$HOST" "${cl}bash ~/nextech/arm_ik/after_boot.sh"
    ;;
  autostart)
    opt=${1:-}
    case "$opt" in ""|--check|--undo) ;; *) echo "autostart takes nothing, --check or --undo"; exit 1 ;; esac
    exec ssh -t -i "$KEY" -o ConnectTimeout=8 "$HOST" "bash ~/nextech/arm_ik/install_autostart.sh $opt"
    ;;
  dry|send)
    vel=${1:-0.5}; secs=${2:-$([ "$cmd" = send ] && echo 180 || echo 60)}; tool=${3:-hand}
    [[ "$vel" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "max_vel must be a number (rad/s)"; exit 1; }
    [[ "$secs" =~ ^[0-9]+$ ]] || { echo "seconds must be a whole number"; exit 1; }
    case "$tool" in hand|drill) ;; *) echo "tool must be hand or drill"; exit 1 ;; esac
    temp=""
    if [ -n "${MAX_TEMP:-}" ]; then
      [[ "$MAX_TEMP" =~ ^([0-9]+|none)$ ]] || { echo "MAX_TEMP must be a whole number (50..70) or none"; exit 1; }
      temp="MAX_TEMP=$MAX_TEMP "
    fi
    for v in TEMP_PAUSE TEMP_RESUME; do
      if [ -n "${!v:-}" ]; then
        [[ "${!v}" =~ ^[0-9]+$ ]] || { echo "$v must be a whole number"; exit 1; }
        temp="${temp}$v=${!v} "
      fi
    done
    if [ -n "${WAIST_CORRECT:-}" ]; then
      case "$WAIST_CORRECT" in roll|pitch|roll,pitch|pitch,roll) ;; *) echo "WAIST_CORRECT must be roll, pitch or roll,pitch"; exit 1 ;; esac
      temp="${temp}WAIST_CORRECT=$WAIST_CORRECT "
    fi
    exec "${SSH[@]}" "${temp}bash ~/nextech/arm_ik/run_live.sh $cmd $vel $secs $tool"
    ;;
  stop)
    exec "${SSH[@]}" "bash ~/nextech/arm_ik/stop.sh"
    ;;
  status)
    exec "${SSH[@]}" 'bash ~/nextech/arm_ik/state.sh 2>&1
for p in 6379:Redis 63901:"PC Service" 13579:camera; do
  ss -ltn 2>/dev/null | grep -qE ":${p%%:*}\b" && echo "${p#*:} ${p%%:*}: listening" || echo "${p#*:} ${p%%:*}: NOT listening"
done
bash ~/nextech/cam_to_pico/cam_to_pico.sh status 2>&1 | tail -n 3'
    ;;
  ssh)
    exec ssh -t -i "$KEY" "$HOST"
    ;;
  *)
    usage
    ;;
esac
