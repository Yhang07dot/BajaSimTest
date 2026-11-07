from glob import glob
from setuptools import setup

package = 'baja_perception_sim'
setup(
    name=package, version='0.1.0', packages=[package],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/launch', glob('launch/*.launch.py')),
        ('share/' + package + '/config', glob('config/*')),
        ('share/' + package + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Baja simulation team', maintainer_email='maintainer@example.com',
    description='Fixed-route Gazebo perception benchmark', license='MIT',
    entry_points={'console_scripts': [
        'generate = baja_perception_sim.scene:main',
        'platform = baja_perception_sim.platform:main',
        'evaluate = baja_perception_sim.evaluation:main',
        'example_algorithm = baja_perception_sim.example:main',
        'inspect_sensors = baja_perception_sim.inspect:main',
    ]},
)
