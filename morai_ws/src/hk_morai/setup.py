from glob import glob
from setuptools import setup

setup(
    name='hk_morai', version='0.1.0', packages=['hk_morai'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/hk_morai']),
                ('share/hk_morai', ['package.xml']),
                ('share/hk_morai/config', glob('config/*.yaml')),
                ('share/hk_morai/launch', glob('launch/*.py')),
                ('share/hk_morai/rviz', glob('rviz/*.rviz')),
                ('share/hk_morai/course', glob('course/*')),
               ],
    install_requires=['setuptools'], zip_safe=True, maintainer='Taeyeong',
    maintainer_email='taeyeong@users.noreply.github.com',
    description='HEVEN 2026 MORAI base code: localization, planner and control nodes (Stanley + PI).', license='MIT',
    entry_points={'console_scripts': ['localization = hk_morai.localization:main',
                                        'planner = hk_morai.planner:main',
                                        'control = hk_morai.control:main',
                                        'lidar_perception = hk_morai.lidar_perception:main',
                                        'camera_perception = hk_morai.camera_perception:main']},
)
