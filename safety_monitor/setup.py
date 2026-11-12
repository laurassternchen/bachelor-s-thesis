from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'safety_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    scripts=['scripts/record_run.sh'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laurahoppe994',
    maintainer_email='laurahoppe994@gmail.com',
    description='Passive, observe-only SafetyMonitor for Nav2 retrofit comparison',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor_node = safety_monitor.safety_monitor_node:main',
            'send_goal.py = safety_monitor.send_goal:main',
            'run_campaign.py = safety_monitor.run_campaign:main',
        ],
    },
)
