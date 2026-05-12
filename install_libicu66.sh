#!/bin/bash

# 步骤1: 更新包列表
echo "更新包列表..."
sudo apt-get update

# 步骤2: 安装gedit (如果尚未安装)
echo "确保gedit已安装..."
sudo apt-get install -y gedit

# 步骤3: 编辑sources.list文件，添加安全更新源
echo "请在打开的gedit窗口中手动添加以下行到第二行，然后保存并关闭："
echo "deb http://security.ubuntu.com/ubuntu focal-security main"
read -p "按任意键继续..."
sudo gedit /etc/apt/sources.list

# 步骤4: 再次更新包列表并安装libicu66
echo "再次更新包列表并安装libicu66..."
sudo apt-get update
sudo apt-get install libicu66

# 添加删除临时源的逻辑
echo "删除临时安全更新源..."
sudo sed -i '/deb http:\/\/security.ubuntu.com\/ubuntu focal-security main/d' /etc/apt/sources.list

echo "libicu66安装完成，临时源已清理！"
