import os  # os 모듈 임포트
from setuptools import setup

package_name = 'kuka_task_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'rclpy', 'numpy', 'scipy', 'sensor_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='example@domain.com',
    description='Python client for Kuka transform input service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_client = kuka_task_client.task_client:main',
            'visual_command_client = kuka_task_client.visual_command_client:main',
            'kuka_client_joint_update = kuka_task_client.kuka_client_joint_update:main',  # 새로운 노드 추가
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 경로 추가
        (os.path.join('share', package_name, 'launch'), ['launch/kuka_client_joint_update.launch.py']),
    ],
)
