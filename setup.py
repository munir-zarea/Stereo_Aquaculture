from setuptools import setup

package_name = 'mmlove_stereo_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sbs.launch.py']),
        ('share/' + package_name + '/config', ['config/sbs_camera.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Munir Zarea',
    maintainer_email='zaream@hawaii.edu',
    description='Stereo SBS camera recorder',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sbs_camera_node = mmlove_stereo_recorder.sbs_camera_node:main'
        ],
    },
)
