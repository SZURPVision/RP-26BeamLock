#!/usr/bin/env bash
# 开机自动运行 RP-26BeamLock，并在程序崩溃后自动重启。
#
# ⚠️ 本脚本会修改系统配置，请先读完再执行（需要 sudo）：
#   1) 修改 /etc/gdm3/custom.conf 开启图形界面自动登录（原文件备份为 custom.conf.bak）
#   2) 生成 <WorkDir>/start.sh，并在 /etc/systemd/system/ 安装 autostart.service（enable + 立即生效）
#   卸载/还原请运行同目录的 remove_autostart.sh。
#
# 可用环境变量覆盖默认值：
#   BEAMLOCK_USER      运行程序的用户（默认：sudo 调用者，否则当前用户）
#   BEAMLOCK_WORKDIR   程序工作目录（默认：本仓库的 build 目录）
#   BEAMLOCK_START_CMD 启动命令（默认：./App/beamlock）
#   BEAMLOCK_SERIAL    需要放开权限的串口设备（默认：/dev/ttyACM0）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SerialName="${BEAMLOCK_SERIAL:-/dev/ttyACM0}"
WorkDir="${BEAMLOCK_WORKDIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)/build}"
StartCommand="${BEAMLOCK_START_CMD:-./App/beamlock}"
User="${BEAMLOCK_USER:-${SUDO_USER:-$(id -un)}}"

echo "将安装开机自启："
echo "  用户     : $User"
echo "  工作目录 : $WorkDir"
echo "  启动命令 : $StartCommand"
echo "  串口设备 : $SerialName"
echo

if [ ! -d "$WorkDir" ]; then
    echo "错误：工作目录不存在（$WorkDir）。请先构建，或通过 BEAMLOCK_WORKDIR 指定。" >&2
    exit 1
fi

cd "$WorkDir" || exit 1

echo "Setting auto login for user $User..."
# check if /etc/gdm3/custom.conf exist
if [ -f /etc/gdm3/custom.conf ]; then
    # using gdm3, file is /etc/gdm3/custom.conf
    sudo cp -a /etc/gdm3/custom.conf /etc/gdm3/custom.conf.bak
    sudo tee /etc/gdm3/custom.conf > /dev/null <<EOF
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=$User
[security]
[xdmcp]
[chooser]
[debug]
EOF
else
    echo "Error: /etc/gdm3/custom.conf not found. Please set up auto login manually."
fi

echo "Generating startup script..."
echo "#!/bin/bash" > start.sh
echo "cd $WorkDir" >> start.sh
echo "if [ -e $SerialName ]; then chmod 666 $SerialName; fi" >> start.sh
echo "exec runuser -u $User -- $StartCommand" >> start.sh
chmod +x start.sh

echo "Generating systemd service file..."
echo "[Unit]" > autostart.service
echo "Description=Autostart" >> autostart.service
echo "" >> autostart.service
echo "[Service]" >> autostart.service
echo "Type=simple" >> autostart.service
echo "ExecStart=$WorkDir/start.sh" >> autostart.service
echo "Restart=on-failure" >> autostart.service
echo "RestartSec=2" >> autostart.service
echo "" >> autostart.service
echo "[Install]" >> autostart.service
echo "WantedBy=multi-user.target" >> autostart.service

echo "Moving service file to systemd directory..."
sudo mv autostart.service /etc/systemd/system/
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload
echo "Enabling service..."
sudo systemctl enable autostart.service

echo "Autostart setup completed. Please reboot the system to apply changes."
