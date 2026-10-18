# setup.py: instrucciones para instalar este paquete Python de ROS 2 (lo usa "colcon build").
import os
from glob import glob

from setuptools import setup

package_name = 'andesrobot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # Archivos que NO son código Python y se copian a install/<paquete>/share/<paquete>/.
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
    description='Navegación autónoma (Nav2, modelo Omni) de MARTHA con waypoints con nombre.',
    license='Apache-2.0',
    # Ejecutables: 'nombre = modulo:funcion' -> ros2 run andesrobot_navigation <nombre>
    entry_points={'console_scripts': [
        # ros2 run andesrobot_navigation ir_a sofa
        'ir_a = andesrobot_navigation.ir_a:main',
        # publica los waypoints como marcadores en RViz
        'marcadores = andesrobot_navigation.marcadores:main',
    ]},
)
