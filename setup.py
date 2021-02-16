from setuptools import setup

package_name = 'sig_loreal_picking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ribi1011',
    maintainer_email='ribi1011@hs-karlsruhe.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'picking_node = sig_loreal_picking.picking_node:main'
        ],
    },
)
