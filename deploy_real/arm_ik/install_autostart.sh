#!/bin/bash
# Run ONCE on the G1 (asks for the sudo password once). From the operator PC: bash g1.sh autostart
# After this, `g1.sh up` (after_boot.sh) needs no password, so it can also be run by Claude over SSH.
# Nothing here moves the robot, and no permission (sudoers, groups, SSH) is changed.
#   1) Docker: the containers nextech-redis (Redis) and xrobot-service (PC Service) get restart=unless-stopped, so
#      Docker starts them by itself every time the G1 boots (until someone runs `docker stop` on them).
#      The Docker service itself must start at boot (systemctl is-enabled docker); it is enabled if it is not.
#   2) udev rule /etc/udev/rules.d/90-nextech-head-camera-power.rules: USB power saving off for the head camera
#      (Intel, USB 2-2.3) and its hub (2-2) whenever they appear (boot, USB reset). Same as after_boot.sh step 2,
#      which until now was lost on every reboot. It is also applied once now.
#   bash install_autostart.sh            install (the two above)
#   bash install_autostart.sh --check    show the current state (read only; the restart policy needs sudo to read)
#   bash install_autostart.sh --undo     restart=no again and remove the udev rule
set -u
CONTAINERS=(nextech-redis xrobot-service)
RULE=/etc/udev/rules.d/90-nextech-head-camera-power.rules
CAM_USB=2-2.3
HUB_USB=2-2
mode=${1:-install}

rule_text() {
  cat <<'EOF'
# NexTech 2026 teleop (TWIST2 deploy_real/arm_ik/install_autostart.sh): USB power saving off for the G1 head camera
# (Intel RealSense at USB 2-2.3) and its hub (2-2), whenever they appear. Undo: bash install_autostart.sh --undo
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", KERNEL=="2-2.3", ATTR{idVendor}=="8086", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", KERNEL=="2-2", TEST=="power/control", ATTR{power/control}="on"
EOF
}

show() {
  echo "docker service at boot: $(systemctl is-enabled docker 2>/dev/null || echo unknown)"
  for c in "${CONTAINERS[@]}"; do
    p=$(sudo -n docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$c" 2>/dev/null) || p="(needs sudo to read)"
    echo "container $c: restart=${p:-no}"
  done
  [ -e "$RULE" ] && echo "udev rule: $RULE present" || echo "udev rule: $RULE missing"
  for d in "$CAM_USB" "$HUB_USB"; do
    f=/sys/bus/usb/devices/$d/power/control
    [ -e "$f" ] && echo "USB $d ($(cat /sys/bus/usb/devices/$d/product 2>/dev/null)): power/control=$(cat "$f")"
  done
}

case "$mode" in
  --check)
    show
    exit 0
    ;;
  install|--undo)
    ;;
  *)
    sed -n 2,14p "$0"
    exit 1
    ;;
esac

for c in "${CONTAINERS[@]}"; do
  sudo docker inspect "$c" > /dev/null || { echo "ABORT: no Docker container named $c (nothing changed)"; exit 1; }
done

if [ "$mode" = --undo ]; then
  sudo docker update --restart no "${CONTAINERS[@]}" > /dev/null && echo "containers: restart=no (start them with g1.sh up)"
  if [ -e "$RULE" ]; then
    sudo rm -f "$RULE" && sudo udevadm control --reload-rules && echo "removed $RULE"
  fi
  show
  exit 0
fi

echo "== 1/2 Redis and PC Service start with the G1"
sudo docker update --restart unless-stopped "${CONTAINERS[@]}" > /dev/null || { echo "docker update failed"; exit 1; }
if [ "$(systemctl is-enabled docker 2>/dev/null)" != enabled ]; then
  echo "the docker service was not enabled at boot: enabling it"
  sudo systemctl enable docker
fi

echo "== 2/2 head camera USB power saving off (udev rule)"
rule_text | sudo tee "$RULE" > /dev/null || { echo "could not write $RULE"; exit 1; }
sudo udevadm control --reload-rules
if [ "$(cat /sys/bus/usb/devices/$CAM_USB/idVendor 2>/dev/null)" = "8086" ]; then
  for d in "$CAM_USB" "$HUB_USB"; do
    f=/sys/bus/usb/devices/$d/power/control
    [ -e "$f" ] && echo on | sudo tee "$f" > /dev/null
  done
fi

echo
show
echo
echo "done. From the next boot, Redis and PC Service start by themselves and g1.sh up asks for no password."
