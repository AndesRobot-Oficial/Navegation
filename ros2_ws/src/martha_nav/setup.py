# Instala solo la parte de la tesis que corre en el robot (ver README.md). Sin ejecutables propios:
# el nodo que usa esta lógica es andesrobot_navigation/ppo_controller.py.
from setuptools import find_packages, setup

package_name = 'martha_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mocolax',
    maintainer_email='nicolas58david@hotmail.com',
    description='Copia de la lógica del planificador local PPO de la tesis (solo numpy).',
    license='MIT',
)
