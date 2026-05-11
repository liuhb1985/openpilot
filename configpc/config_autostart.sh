#!/bin/bash

echo "copy logo.ico"
cp logo.ico /home/$LOGNAME/
echo "copy Carrotpilot.desktop"
sudo cp Carrotpilot.desktop /usr/share/applications/
echo "chmod +x Carrotpilot.desktop"
sudo chmod +x /usr/share/applications/Carrotpilot.desktop
#增加启动脚本
echo "copy launch_pc.sh"
cp launch_pc.sh /home/$LOGNAME/
echo "chmod +x launch_pc.sh"
chmod +x /home/$LOGNAME/launch_pc.sh
#增加开机启动脚本
if [ ! -d /home/$LOGNAME/.config/autostart ]; then
    echo "mkdir .config/autostart"
    mkdir -p /home/$LOGNAME/.config/autostart
fi
echo "copy autostart_launch_pc.sh.desktop"
cp autostart_launch_pc.sh.desktop /home/$LOGNAME/.config/autostart/
chmod +x /home/$LOGNAME/.config/autostart/autostart_launch_pc.sh.desktop
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
# 让当前用户获得串口（/dev/ttyUSB0、/dev/ttyACM0等）访问权限
sudo usermod -a -G dialout $LOGNAME
echo "config autostart end"
