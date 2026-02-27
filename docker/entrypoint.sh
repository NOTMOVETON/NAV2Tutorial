#!/usr/bin/env bash
set -euo pipefail

PUID=$(id -u $DEV_USER)
PGID=$(id -g $DEV_USER)

: "${DEV_USER:?DEV_USER must be set}"
readonly DEV_USER

echo "===== ENTRYPOINT START ====="
echo "[INFO] UID        =   $(id -u)"
echo "[INFO] GID        =   $(id -g)"
echo "[INFO] User       =   $(whoami)"
echo "[INFO] Workdir    =   $(pwd)"
echo "       -------------        "
echo "[INFO] PUID       =   $PUID"
echo "[INFO] PGID       =   $PGID"
echo "[INFO] DEV_USER   =   $DEV_USER"
echo "       -------------        "
echo "[INFO] ROS_DISTRO =   $ROS_DISTRO"
echo "============================"
echo

# --- Fix volume ownership ---
for path in /ros2_ws/build /ros2_ws/install /ros2_ws/log; do
  if [ -d "$path" ]; then
    echo "[DEBUG] Fixing ownership for $path → ${PUID}:${PGID}"
    chown -R "${PUID}:${PGID}" "$path" || true
  fi
done

echo "============================"
echo "[INFO] After chown check:"
ls -ld /ros2_ws /ros2_ws/build /ros2_ws/install /ros2_ws/log || true
echo "============================"

# --- Add auto-switch to dev for future exec sessions ---
if [ "$(id -u)" = "0" ]; then
  echo "[INFO] Ensuring auto-switch to ${DEV_USER} in future interactive sessions..."
  if ! grep -q "gosu ${DEV_USER} bash" /root/.bashrc 2>/dev/null; then
    echo "if [ \"\$(id -u)\" = \"0\" ]; then exec gosu ${DEV_USER} bash; fi" >> /root/.bashrc
  fi
fi

# --- Switch to dev or run command ---
if [ "$(id -u)" = "0" ]; then
  if [ $# -eq 0 ]; then
    echo "[INFO] Switching to ${DEV_USER}..."
    echo
    exec gosu "${DEV_USER}" bash -l
  else
    echo "[INFO] Switching to ${DEV_USER} and running command..."
    echo "[INFO] Command: $*"
    exec gosu "${DEV_USER}" bash -lc "$*"
  fi
else
  echo "[INFO] Running command as $(id -u)..."
  echo "[INFO] Command: $*"
  exec "$@"
fi
