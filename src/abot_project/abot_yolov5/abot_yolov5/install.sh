python3 -m pip install --upgrade pip
pip3 install matplotlib pillow pyyaml tensorboard tqdm scipy seaborn -i https://pypi.tuna.tsinghua.edu.cn/simple 
pip3 install torch==1.8.0
pip3 install Cython
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev 

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.9.0
python3 setup.py install --user


#install labelimg
sudo apt install pyqt5*
sudo apt install qt5-default qttools5-dev-tools
pip3 install pyqt5
pip3 install labelimg


