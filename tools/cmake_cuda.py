import subprocess, os

home = os.path.expanduser('~')
d = home + '/opencv-4.5.5'
contrib = home + '/opencv_contrib-4.5.5/modules'

os.makedirs(d + '/build', exist_ok=True)
os.chdir(d + '/build')

subprocess.run([
    'cmake',
    '-D', 'CMAKE_BUILD_TYPE=RELEASE',
    '-D', 'CMAKE_INSTALL_PREFIX=/usr/local',
    '-D', 'OPENCV_EXTRA_MODULES_PATH=' + contrib,
    # CUDA
    '-D', 'WITH_CUDA=ON',
    '-D', 'WITH_CUDNN=ON',
    '-D', 'CUDA_ARCH_BIN=5.3',
    '-D', 'ENABLE_FAST_MATH=ON',
    '-D', 'CUDA_FAST_MATH=ON',
    '-D', 'WITH_CUBLAS=ON',
    # Python
    '-D', 'BUILD_opencv_python3=ON',
    '-D', 'BUILD_opencv_python2=ON',
    # 图像格式（不需要）
    '-D', 'WITH_OPENEXR=OFF',
    '-D', 'WITH_JASPER=OFF',
    # 砍掉不需要的模块
    '-D', 'BUILD_opencv_wechat_qrcode=OFF',
    '-D', 'BUILD_opencv_xfeatures2d=OFF',
    '-D', 'BUILD_opencv_face=OFF',
    '-D', 'BUILD_opencv_text=OFF',
    '-D', 'BUILD_opencv_dnn=OFF',
    '-D', 'BUILD_opencv_bgsegm=OFF',
    '-D', 'BUILD_opencv_bioinspired=OFF',
    '-D', 'BUILD_opencv_ccalib=OFF',
    '-D', 'BUILD_opencv_datasets=OFF',
    '-D', 'BUILD_opencv_dpm=OFF',
    '-D', 'BUILD_opencv_fuzzy=OFF',
    '-D', 'BUILD_opencv_hfs=OFF',
    '-D', 'BUILD_opencv_line_descriptor=OFF',
    '-D', 'BUILD_opencv_optflow=OFF',
    '-D', 'BUILD_opencv_phase_unwrapping=OFF',
    '-D', 'BUILD_opencv_plot=OFF',
    '-D', 'BUILD_opencv_reg=OFF',
    '-D', 'BUILD_opencv_rgbd=OFF',
    '-D', 'BUILD_opencv_saliency=OFF',
    '-D', 'BUILD_opencv_shape=OFF',
    '-D', 'BUILD_opencv_stereo=OFF',
    '-D', 'BUILD_opencv_structured_light=OFF',
    '-D', 'BUILD_opencv_superres=OFF',
    '-D', 'BUILD_opencv_surface_matching=OFF',
    '-D', 'BUILD_opencv_tracking=OFF',
    '-D', 'BUILD_opencv_videostab=OFF',
    '-D', 'BUILD_opencv_xobjdetect=OFF',
    '-D', 'BUILD_opencv_xphoto=OFF',
    '-D', 'BUILD_opencv_aruco=OFF',
    '-D', 'BUILD_opencv_quality=OFF',
    '-D', 'BUILD_opencv_gapi=OFF',
    # 不编测试/示例
    '-D', 'BUILD_TESTS=OFF',
    '-D', 'BUILD_PERF_TESTS=OFF',
    '-D', 'BUILD_EXAMPLES=OFF',
    '-D', 'BUILD_opencv_apps=OFF',
    '-D', 'WITH_TBB=ON',
    '-D', 'WITH_V4L=ON',
    '..',
])
