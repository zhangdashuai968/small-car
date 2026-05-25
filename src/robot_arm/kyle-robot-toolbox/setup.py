
#!/usr/bin/env python3

import setuptools
from setuptools import find_packages

setuptools.setup(name='kyle_robot_toolbox',
                 version='0.0.2',
                 description="Kyle Robot Toolbox",
                 author='Shunkai Xing',
                 author_email='xingshunkai@qq.com',
                 long_description=open("README.rst", "r", encoding="utf-8").read(),
                 long_description_content_type = 'text/markdown',
                 url="https://gitee.com/robokyle/kyle-robot-toolbox",
                 packages=find_packages(exclude=["example"]),
                 classifiers="""
                 Development Status :: 4 - Beta
                 Programming Language :: Python :: 3 :: Only
                 Programming Language :: Python :: 3.6
                 Programming Language :: Python :: 3.7
                 Programming Language :: Python :: 3.8
                 Programming Language :: Python :: 3.9
                 Programming Language :: Python :: 3.10
                 License :: OSI Approved :: MIT License
                 Operating System :: OS Independent
                 Operating System :: Microsoft :: Windows
                 Operating System :: POSIX
                 Operating System :: Unix
                 Operating System :: MacOS
                 Topic :: Scientific/Engineering
                 Topic :: Education
                 Topic :: Documentation
                 Topic :: Home Automation
                 Topic :: Scientific/Engineering :: Artificial Intelligence
                 Topic :: Scientific/Engineering :: Electronic Design Automation (EDA)
                 Topic :: Scientific/Engineering :: Image Recognition
                 Topic :: Software Development :: Embedded Systems
                 Topic :: Software Development :: Version Control :: Git
                 Intended Audience :: Education
                 Intended Audience :: Science/Research
                 Intended Audience :: Manufacturing
                 Intended Audience :: Developers
                 """.splitlines(),
                 python_requires='>=3.6',
                 install_requires=[
                     'numpy',
                     'matplotlib',
                     'PyYaml', 
                     'inputs', 
                    #  'cv2',
                    #  'open3d',
                    #  'pybullet', 
                 ],
                 
                #  package_dir={
                #      'kyle_robot_toolbox': 'kyle_robot_toolbox'
                #  },
                 package_data={
                    'kyle_robot_toolbox': ['resources/*'], 
                 }
)