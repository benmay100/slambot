from setuptools import find_packages, setup

package_name = 'slambot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- NEW ENTRY for the JSON file ---
        # The installation path: 'share/slambot_controllers/data'
        ('share/' + package_name + '/data', ['data/qr_codes_coordinates.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben-may',
    maintainer_email='benalexandermay@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_code_reader = slambot_controllers.qr_code_reader:main',
            'slambot_nav2_controller = slambot_controllers.slambot_nav2_controller:main'
        ],
    },
)
