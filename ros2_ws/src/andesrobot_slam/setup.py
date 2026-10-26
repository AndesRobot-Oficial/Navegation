# setup.py: instrucciones para instalar este paquete Python de ROS 2 (lo usa "colcon build").
import os
from glob import glob

from setuptools import setup

package_name = 'andesrobot_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # Archivos que NO son código Python y se copian a install/<paquete>/share/<paquete>/.
    # Los mapas NO van aquí (a diferencia de milo_ws): viven en martha_ws/maps/, que el
    # contenedor ve como /maps, para que ./sim.sh nav y ./robot.sh nav los usen sin recompilar.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mocolax',
    maintainer_email='nicolas58david@hotmail.com',
    description='Mapeo 2D de MARTHA con slam_toolbox, en Gazebo y en el robot real.',
    license='Apache-2.0',
    # Sin programas propios: solo launches y parámetros.
    entry_points={'console_scripts': []},
)
