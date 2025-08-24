from setuptools import setup

package_name = 'k9_chess_interfaces'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Richard Hopkins',
    author_email='richard@example.com',
    description='ROS 2 interfaces (actions, services, messages) for K9 chess robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)