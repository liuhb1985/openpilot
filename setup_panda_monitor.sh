#!/bin/bash
# 一键安装 Panda 自动关机服务

# 1 禁止系统合盖挂起
echo "修改 /etc/systemd/logind.conf，禁止挂起..."
sudo sed -i 's/^#HandleLidSwitch=.*/HandleLidSwitch=ignore/' /etc/systemd/logind.conf
sudo sed -i 's/^#HandleLidSwitchExternalPower=.*/HandleLidSwitchExternalPower=ignore/' /etc/systemd/logind.conf
sudo sed -i 's/^#HandleLidSwitchDocked=.*/HandleLidSwitchDocked=ignore/' /etc/systemd/logind.conf
sudo systemctl restart systemd-logind
echo "系统挂起设置已更新"

# 2 创建 panda_monitor.service 文件
SERVICE_FILE="/etc/systemd/system/panda_monitor.service"
SCRIPT_PATH="/data/openpilot/panda_monitor.py"
echo "创建 systemd service 文件..."
sudo bash -c "cat > $SERVICE_FILE" <<EOL
[Unit]
Description=Panda Device Monitor
After=network.target

[Service]
User=root
ExecStart=/usr/bin/python3 $SCRIPT_PATH
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOL

# 3 设置脚本权限
echo "设置脚本权限..."
sudo chown op:op $SCRIPT_PATH
sudo chmod +x $SCRIPT_PATH

# 4 重新加载 systemd 并启用服务
echo "启用并启动 panda_monitor 服务..."
sudo systemctl daemon-reload
sudo systemctl enable panda_monitor.service
sudo systemctl start panda_monitor.service

# 5 完成
echo "一键安装完成！"
echo "查看服务状态: sudo systemctl status panda_monitor.service"
