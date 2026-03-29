#!/bin/bash

#增加启动脚本
echo "copy launch_cam_8845.sh"
cp launch_cam_8845.sh /home/$LOGNAME/
echo "chmod +x launch_cam_8845.sh"
chmod +x /home/$LOGNAME/launch_cam_8845.sh
#增加开机启动脚本
if [ ! -d /home/$LOGNAME/.config/autostart ]; then
    echo "mkdir .config/autostart"
    mkdir -p /home/$LOGNAME/.config/autostart
fi
echo "copy autostart_launch_cam_8845.sh.desktop"
cp autostart_launch_cam_8845.sh.desktop /home/$LOGNAME/.config/autostart/
chmod +x /home/$LOGNAME/.config/autostart/autostart_launch_cam_8845.sh.desktop
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
echo "config autostart end"
