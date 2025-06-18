from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'yolov8_obb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装ckpt文件
        (os.path.join('share', package_name, 'ckpt'), glob('ckpt/*.pt')),
        # 安装img文件
        (os.path.join('share', package_name, 'img'), glob('img/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dod',
    maintainer_email='319377758@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bolt_det_pub = yolov8_obb.bolt_det_pub:main',
            'target_pos_pub = yolov8_obb.target_pos_pub:main',
        ],
    },
)
