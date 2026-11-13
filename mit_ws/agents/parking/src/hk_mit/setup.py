from glob import glob
from setuptools import setup

setup(
    name='hk_mit', version='0.1.0', packages=['hk_mit'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/hk_mit']),
                ('share/hk_mit', ['package.xml']),
                ('share/hk_mit/config', glob('config/*.yaml')),
                ('share/hk_mit/launch', glob('launch/*.py')),
                ('share/hk_mit/rviz', glob('rviz/*.rviz')),
                ('share/hk_mit/course', glob('course/*')),
               ],
    install_requires=['setuptools'], zip_safe=True, maintainer='Taeyeong',
    maintainer_email='taeyeong@users.noreply.github.com',
    description='HEVEN 2026 MIT base code: localization, planner and control nodes (Stanley + PI).', license='MIT',
    entry_points={'console_scripts': ['localization = hk_mit.localization:main',
                                        'planner = hk_mit.planner:main',
                                        'control = hk_mit.control:main',
                                        'lidar_perception = hk_mit.lidar_perception:main']},
)
