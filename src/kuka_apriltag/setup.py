from setuptools import setup
import os
from glob import glob

package_name = 'kuka_apriltag'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # urdf 및 meshes 디렉토리의 파일들을 설치할 경로 지정
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # launch 디렉토리와 그 안의 파일들을 설치할 경로 지정
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='kuka_apriltag description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행할 Python 스크립트를 여기 추가할 수 있습니다.
        ],
    },
)
