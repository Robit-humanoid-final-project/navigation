from glob import glob

from setuptools import setup

package_name = 'robit_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cgjeong77',
    maintainer_email='todo@todo.todo',
    description='ROBIT 21기 휴머노이드 내비게이션: A-star 장애물 회피 + PID 경로 추종',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigation_node = robit_navigation.navigation_node:main',
            'fake_robot_node = robit_navigation.fake_robot_node:main',
            'nav_gui_node = robit_navigation.nav_gui_node:main',
        ],
    },
)
