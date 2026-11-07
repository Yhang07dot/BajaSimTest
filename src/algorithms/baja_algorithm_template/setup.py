from glob import glob
from setuptools import setup

package = 'baja_algorithm_template'
setup(
    name=package, version='0.1.0', packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Baja algorithm team', maintainer_email='maintainer@example.com',
    description='Perception integration template', license='MIT',
    entry_points={'console_scripts': ['perception = baja_algorithm_template.node:main']},
)
