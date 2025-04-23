from setuptools import setup

package_name = 'trajectory_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packges',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ton_nom',
    maintainer_email='ton_email@example.com',
    description='Ton descriptif ici',
    license='propriétaire',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_trajectory = trajectory_control.send_trajectory:main',  # Ligne à vérifier
            'reset_position = trajectory_control.reset_position:main',  # Ligne à vérifier*
            'suite_de_positions = trajectory_control.suite_de_positions:main',  # Ligne à vérifier
        ],
    },
)
