from setuptools import find_packages, setup

package_name = 'motor_control_package'

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
    maintainer='rcameron',
    maintainer_email='astoriacamfam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "left_tread = motor_control_package.left_tread:main",
            "right_tread = motor_control_package.right_tread:main",
            "arm_joint = motor_control_package.arm_joint:main",
            "bucket_joint = motor_control_package.bucket_joint:main"
        ],
    },
)
