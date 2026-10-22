from glob import glob
from setuptools import setup
package_name = 'shopping_zone_manager'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])] +
                 [('share/' + package_name + '/' + d, glob(d + '/*')) for d in ['config', 'launch']],
      install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
      maintainer='2006wu', maintainer_email='2006wu@users.noreply.github.com',
      description='Simulation shopping robot logic', license='Apache-2.0',
      entry_points={'console_scripts': ['zone_navigator = shopping_zone_manager.node:main', 'goto_zone = shopping_zone_manager.cli:goto_main', 'list_zones = shopping_zone_manager.cli:list_main']})
