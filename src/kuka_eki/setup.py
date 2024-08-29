from setuptools import setup

package_name = 'kuka_eki'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/kuka_eki_interface_bringup.launch.py',
            'launch/kuka_eki_joint_initial.launch.py',
            'launch/kuka_bringup.launch.py'  # 새로 추가된 런치 파일
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuka',
    maintainer_email='tlatmdgus19589@naver.com',
    description='Service servers for Kuka control box.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kuka_eki_task_server = kuka_eki.kuka_eki_task_server:main',
            'kuka_eki_joint_server = kuka_eki.kuka_eki_joint_server:main',
            'joint_state_publisher = kuka_eki.joint_state_publisher:main',
        ],
    },
)
