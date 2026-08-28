#!/usr/bin/env bash
set -euo pipefail

export USER="${USER_NAME:-ros}"
export HOME="${HOME:-/home/${USER}}"
export DISPLAY="${VNC_DISPLAY:-:1}"

VNC_DIR="${HOME}/.vnc"
mkdir -p "${VNC_DIR}"
chmod 700 "${VNC_DIR}"

if [[ -z "${VNC_PASSWORD:-}" ]]; then
  echo "VNC_PASSWORD must not be empty." >&2
  exit 1
fi

if [[ "${#VNC_PASSWORD}" -gt 8 ]]; then
  echo "TigerVNC only uses the first 8 characters of VNC_PASSWORD." >&2
fi

printf '%s\n' "${VNC_PASSWORD}" | vncpasswd -f > "${VNC_DIR}/passwd"
chmod 600 "${VNC_DIR}/passwd"

cat > "${VNC_DIR}/xstartup" <<'EOF'
#!/usr/bin/env bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XKL_XMODMAP_DISABLE=1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

xsetroot -solid '#263238'

if command -v startxfce4 >/dev/null 2>&1; then
  exec dbus-launch --exit-with-session startxfce4
fi

xterm -geometry 120x32+40+40 -ls -title "ROS 2 Humble" &
exec x-window-manager
EOF
chmod +x "${VNC_DIR}/xstartup"

vncserver -kill "${DISPLAY}" >/dev/null 2>&1 || true
rm -f "/tmp/.X${DISPLAY#:}-lock" "/tmp/.X11-unix/X${DISPLAY#:}"

cleanup() {
  vncserver -kill "${DISPLAY}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

vncserver "${DISPLAY}" \
  -geometry "${VNC_GEOMETRY:-1280x800}" \
  -depth "${VNC_DEPTH:-24}" \
  -localhost no \
  -SecurityTypes VncAuth \
  -AlwaysShared

LOG_FILE="${VNC_DIR}/$(hostname)${DISPLAY}.log"
echo "TigerVNC is running on ${DISPLAY}, TCP port ${VNC_PORT:-5901}."
echo "Connect to localhost:${VNC_PORT:-5901} with password '${VNC_PASSWORD}'."
echo "ROS 2 Humble is sourced in new terminals."

touch "${LOG_FILE}"
tail -F "${LOG_FILE}" &
wait $!
