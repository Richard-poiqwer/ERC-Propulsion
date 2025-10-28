from setuptools import setup
import os

package_name = 'odrive_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    include_package_data=True,
    zip_safe=True,
)
