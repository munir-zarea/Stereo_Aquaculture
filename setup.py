from setuptools import setup, find_packages

package_name = 'mmlove_stereo_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stereo.launch.py']),
        ('share/' + package_name + '/config', ['config/stereo_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Munir Zarea',
    maintainer_email='zaream@hawaii.edu',
    description='MMLove stereo camera launcher & rosbag recording',
    license='Apache-2.0',
)
