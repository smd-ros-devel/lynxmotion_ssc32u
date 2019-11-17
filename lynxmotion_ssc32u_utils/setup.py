from setuptools import setup

package_name = 'lynxmotion_ssc32u_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Matt Richard',
    author_email='matthew.w.richard@gmail.com',
    maintainer='Matt Richard',
    maintainer_email='matthew.w.richard@gmail.com',
    description='Utilities for the Lynxmotion SSC-32 servo controller.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ssc32u_shell = lynxmotion_ssc32u_utils.ssc32u_shell:main',
        ],
    },
)
