from setuptools import setup
import os
from glob import glob

package_name = 'mp_input'
submodules = "mp_input/lmks_data"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='jl6017@columbia.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nn_node = nn_computer.nncomputer:main",
            "eye_node = eye_cmds.eyecmds:main",
            "mouth_node = mouth_cmds.mouthcmds:main",
            "neck_node = neck_cmds.neckcmds:main",
            "eyeball_node = eyeball_cmds.eyeballcmds:main",
            "mpinput_node = mp_input.mpinput:main",
        ],
    },
)
