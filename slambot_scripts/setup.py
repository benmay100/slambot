from setuptools import find_packages, setup

package_name = 'slambot_scripts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- NEW ENTRY for the JSON file ---
        # The installation path: 'share/slambot_scripts/data'
        ('share/' + package_name + '/data', ['data/qr_codes_coordinates.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben-may',
    maintainer_email='benalexandermay@gmail.com',
    description='Python scripts for QR code capture and waypoint following on the Slambot platform.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_code_reader = slambot_scripts.qr_code_reader:main',
            'qr_code_waypoint_follower = slambot_scripts.qr_code_waypoint_follower:main'
        ],
    },
)
