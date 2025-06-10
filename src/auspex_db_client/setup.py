from setuptools import find_packages, setup

package_name = 'auspex_db_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Börn Döschl',
    maintainer_email='bjoern.doeschl@unibw.de',
    description='A client interface for the database.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
