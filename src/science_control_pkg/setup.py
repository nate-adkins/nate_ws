from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'science_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nate',
    maintainer_email='nathanpadkins@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = science_control_pkg.ui:main',
        ],
    },
)
