from setuptools import setup
import os                 
from glob import glob     

package_name = 'cobra_motor_control_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Ajoutez cette ligne
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobra',
    maintainer_email='cobra@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = cobra_motor_control_py.motor_controller:main',
        ],
    },
)

