from setuptools import setup

package_name = 'visualizer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data',
            ['../data/points.csv', '../data/positions.csv', '../data/seen.csv', '../data/estimate.csv'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='northeus',
    maintainer_email='xlabuda2@fi.muni.cz',
    description='Rviz2 visualizer from file.',
    license='GNU3',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'visualize = visualizer.publisher:main'
        ],
    },
)
