#!/bin/bash

# 激活conda环境
source $(conda info --base)/etc/profile.d/conda.sh
conda activate 39

# 启动vl_locate.launch
roslaunch vl_locate vl_locate.launch