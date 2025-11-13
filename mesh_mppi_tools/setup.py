from setuptools import find_packages, setup

package_name = 'mesh_mppi_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justus Braun',
    maintainer_email='jubraun@uos.de',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_control_sequence = mesh_mppi_tools.plot_control_sequence:main'
        ],
    },
)
