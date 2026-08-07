from glob import glob

from setuptools import setup

package_name = 'operator_console'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The UI is served from the share directory at runtime, not embedded in the
        # Python package, so it can be edited without reinstalling.
        ('share/' + package_name + '/web', glob('web/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    # Signals colcon to run these tests with pytest. Without it, colcon falls back to
    # unittest discovery, which finds none of the pytest-style test functions and
    # reports "Ran 0 tests" as a pass -- a green CI that tests nothing.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='wai-ming',
    maintainer_email='waiming@galeelee.com',
    description='Web operator console for the Jetson perception stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'console_node = operator_console.console_node:main',
        ],
    },
)
