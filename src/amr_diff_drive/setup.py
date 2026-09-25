from glob import glob

from setuptools import setup

package_name = 'amr_diff_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saswata',
    maintainer_email='saswata27education@gmail.com',
    description='Differential drive controller for the AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_controller = amr_diff_drive.diff_drive_controller:main',
        ],
    },
)
