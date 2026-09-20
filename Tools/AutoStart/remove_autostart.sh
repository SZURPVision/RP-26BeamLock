#!/usr/bin/env bash
set -euo pipefail

echo "Reset auto login..."
# check if /etc/gdm3/custom.conf.bak exist
if [ -f /etc/gdm3/custom.conf.bak ]; then
    # using gdm3, file is /etc/gdm3/custom.conf
    sudo cp -a /etc/gdm3/custom.conf.bak /etc/gdm3/custom.conf
    sudo rm /etc/gdm3/custom.conf.bak
else
    echo "Error: /etc/gdm3/custom.conf.bak not found."
fi

echo "Removing systemd service file..."
if [ -f /etc/systemd/system/autostart.service ]; then
    sudo rm /etc/systemd/system/autostart.service
    echo "Reloading systemd daemon..."
    sudo systemctl daemon-reload
    echo "Disabling service..."
    sudo systemctl disable autostart.service
else
    echo "Error: /etc/systemd/system/autostart.service not found."
fi

echo "autostart setup removed. Please reboot the system to apply changes."
