PC版教程地址
https://gitee.com/huheas/pilotosinit/blob/master/README.md

设置代理
export http_proxy=http://192.168.1.10:7897
export https_proxy=http://192.168.1.10:7897

export http_proxy=http://192.168.22.86:7897
export https_proxy=http://192.168.22.86:7897

安装git和ssh服务器
sudo apt update
sudo apt install git -y
git --version

sudo apt update
sudo apt install openssh-server -y
systemctl status ssh
sudo systemctl start ssh
sudo systemctl enable ssh

根目录下创建/data文件夹
使用git命令clone cpv9-pc仓库
git clone -b cpv9-pc https://jihulab.com/fishop/openpilot.git

------------------------------------------------------------
-->下面为环境搭建教程
备份原始源文件
sudo mv /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak

如果要恢复
sudo cp -f /etc/apt/sources.list.d/ubuntu.sources.bak /etc/apt/sources.list.d/ubuntu.sources

以下为1条命令，全部一起复制粘贴到终端运行，该命令会提示输入当前用户的密码并确认用户有sudo权限和密码正确，否则会提示无权限
sudo bash -c 'cat <<EOF > /etc/apt/sources.list.d/ubuntu.sources
Types: deb
URIs: https://mirrors.aliyun.com/ubuntu/
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
Types: deb
URIs: https://mirrors.aliyun.com/ubuntu/
Suites: noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF'

python配置
export UV_Installer_GHE__Base_URL="https://ghfast.top/https://github.com"
export UV_INDEX_URL=https://mirrors.aliyun.com/pypi/simple
export EXTRA_INDEX_URL=https://mirrors.aliyun.com/pypi/simple
export UV_HTTP_TIMEOUT=600
export PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple
export PIP_TRUSTED_HOST=mirrors.aliyun.com

GPU驱动安装

安装和编译Openpilot
cd openpilot
tools/op.sh setup
uv sync --all-extras #强制安装

如果出现下面的报错：
下列软件包有未满足的依赖关系：libglib2.0-0t64 : 破坏: libglib2.0-0 (< 2.80.0-6ubuntu3~) libncurses5-dev : 依赖: libtinfo6 (= 6.2-0ubuntu2) 但是 6.4+20240113-1ubuntu2 正要被安装依赖: libncurses-dev (= 6.2-0ubuntu2) 但是 6.4+20240113-1ubuntu2 正要被安装 E: 无法修正错误，因为您要求某些软件包保持现状，就是它们破坏了软件包间的依赖关系。↳ [✗] Dependencies installation failed!

则执行下面命令清理一下
sudo sed -i '/focal/d' /etc/apt/sources.list
sudo rm -f /etc/apt/sources.list.d/*focal*.list
sudo apt update

#创建虚拟环境 #激活虚拟环境(不再使用，改用tools/op.sh venv)
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip setuptools wheel --break-system-packages
pip install xattr flask --break-system-packages

# 或使用下面命令进入OP虚拟环境
tools/op.sh venv

#安装opencv开发库
sudo apt update
sudo apt install libopencv-dev
sudo apt install libicu-dev

把openpilot/lib目录下的文件全部拷贝到/usr/lib/x86_64-linux-gnu下


#编译op
tools/op.sh build

# 启动OP,ROAD_CAM指定摄像头,NO_DM驾驶监控开关(注意安全驾驶，关闭驾驶监控仅用测试，正常驾驶请打开驾驶监控功能)
USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 system/manager/manager.py

#如果只是测试一下摄像头显示，使用下面的命令
USE_WEBCAM=1 ROAD_CAM=0 NO_DM=0 selfdrive/debug/uiview.py
参数说明：USE_WEBCAM 使用USB摄像头，需要指定USE_WEBCAM为1，ROAD_CAM=0表示使用编号为0的摄像头（即/dev/video0）, NO_DM=0表示无驾驶员监控
------------------------------------------------------------

git clone https://github.com/ultralytics/yolov5.git
pip install pandas
pip install -r yolov5\requirements.txt

转换yolov5s.pt为onnx格式
有NAVID GPU时
python3 yolov5/export.py --weights yolov5s.pt --img-size 416 --batch-size 1 --dynamic --device 0 --include onnx

只使用CPU渲染
python3 yolov5/export.py --weights yolov5s.pt --img-size 416 --batch-size 1 --dynamic --device cpu --include onnx

固定尺寸416
python3 yolov5/export.py --weights yolov5s.pt --img-size 416 --batch-size 1 --include onnx
或者加--opset 17，AI说比较兼容AMD GPU
python3 yolov5/export.py --weights yolov5s.pt --img-size 416 --batch-size 1 --include onnx --opset 17
固定尺寸640
python3 yolov5/export.py --weights yolov5s.pt --img-size 640 --batch-size 1 --include onnx


可下载官方的onnx
https://huggingface.co/amd/yolov5s/tree/main

安装onnxruntime
下载onnxruntime-linux-x64-1.23.0.tgz并解压至/data/openpilot/third_party/，并改文件夹名称为onnxruntime

把/data/onnxruntime/lib目录下文件全部拷贝到/usr/lib中
sudo cp -r /data/openpilot/third_party/onnxruntime/lib/* /usr/lib

编译代码
g++ -std=c++17 c3cam.cpp -o c3cam -I/data/openpilot/third_party/onnxruntime/include -L/data/openpilot/third_party/onnxruntime/lib -lonnxruntime `pkg-config --cflags --libs opencv4`

列出摄像头和摄像头支持的格式
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext

代码里怎么固定摄像头
可以使用和usb端口相关的设备路径
如/dev/v4l/by-path/

命令列出所有路径
op@ubuntu:/data/openpilot/yolo$ ls -l /dev/v4l/by-path/
总计 0
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usb-0:4:1.0-video-index0 -> ../../video0
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usb-0:4:1.0-video-index1 -> ../../video1
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usb-0:4:1.2-video-index0 -> ../../video2
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usb-0:4:1.2-video-index1 -> ../../video3
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usb-0:7.1:1.0-video-index0 -> ../../video5
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usb-0:7.1:1.0-video-index1 -> ../../video7
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usb-0:7.4.1:1.0-video-index0 -> ../../video8
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usb-0:7.4.1:1.0-video-index1 -> ../../video9
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usbv2-0:4:1.0-video-index0 -> ../../video0
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usbv2-0:4:1.0-video-index1 -> ../../video1
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usbv2-0:4:1.2-video-index0 -> ../../video2
lrwxrwxrwx 1 root root 12 10月  2 19:42 pci-0000:00:14.0-usbv2-0:4:1.2-video-index1 -> ../../video3
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usbv2-0:7.1:1.0-video-index0 -> ../../video5
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usbv2-0:7.1:1.0-video-index1 -> ../../video7
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usbv2-0:7.4.1:1.0-video-index0 -> ../../video8
lrwxrwxrwx 1 root root 12 10月  3 11:15 pci-0000:00:14.0-usbv2-0:7.4.1:1.0-video-index1 -> ../../video9





