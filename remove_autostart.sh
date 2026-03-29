#!/bin/bash

echo "rm logo.ico"
rm -f /home/$LOGNAME/logo.ico
echo "rm sunnypilot.desktop"
sudo rm -f /usr/share/applications/sunnypilot.desktop
echo "rm launch_pc.sh"
rm -f /home/$LOGNAME/launch_pc.sh
echo "rm autostart_launch_pc.sh.desktop"
rm -f /home/$LOGNAME/.config/autostart/autostart_launch_pc.sh.desktop

