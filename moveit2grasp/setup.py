from setuptools import find_packages, setup
import os 
from glob import glob


package_name = 'moveit2grasp'

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
            'grasp_demo = moveit2grasp.grasp_demo:main',
            'simple_grasp = moveit2grasp.simple_grasp:main',
        ],
    },
)
