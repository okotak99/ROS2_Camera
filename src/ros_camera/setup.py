from setuptools import find_packages, setup
import glob
import os

package_name = 'ros_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
        ('share/' + package_name + '/srv', glob.glob(os.path.join('srv', '*.srv')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='dydxkr0410@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = ros_camera.img_publisher:main',
            'camera_server = ros_camera.camera_server:main',
            'filter1 = ros_camera.filter1:main',
            'filter2 = ros_camera.filter2:main'
        ],
    },
)
