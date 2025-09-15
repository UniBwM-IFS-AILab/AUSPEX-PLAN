from setuptools import setup
from glob import glob
import os


package_name = 'auspex_planning'

base_path = os.path.join(os.getcwd(), package_name)
extra_packages = []
extra_packages.append(package_name + '/planner/utils/llm_planner_utils')
extra_packages.append(package_name + '/planner/utils/llm_planner_utils/aems_utils')
extra_packages.append(package_name + '/planner/utils/pattern_planner_utils')
extra_packages.append(package_name + '/planner/utils/alns_utils')
extra_packages.append(package_name + '/planner/utils/up_planner_utils')
extra_packages.append(package_name + '/planner/utils/goal_utils')
extra_packages.append(package_name + '/planner/path_planners/')
extra_packages.append(package_name + '/planner/task_planners/')
extra_packages.append(package_name + '/planner/utils/')


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name+"/planner", package_name+"/interfaces"] + extra_packages,
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*.pddl')),
        (os.path.join("share", package_name, "params"), glob("params/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bjoern Doeschl',
    maintainer_email='bjoern.doeschl@unibw.de',
    description='AUSPEX package for planning.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planning_main_node = auspex_planning.planning_main:main',
            'planning_command_publisher = auspex_planning.pex_cmd_pub:main',
        ],
    },
)
