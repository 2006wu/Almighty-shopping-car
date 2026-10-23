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
      entry_points={'console_scripts': ['cmd_vel_mux = shopping_zone_manager.mode_manager_node:main', 'cmd_vel_watchdog = shopping_zone_manager.output_watchdog_node:main', 'eval_report = shopping_zone_manager.eval_report:main', 'zone_navigator = shopping_zone_manager.node:main', 'goto_zone = shopping_zone_manager.cli:goto_main', 'list_zones = shopping_zone_manager.cli:list_main', 'cancel_zone = shopping_zone_manager.cli:cancel_main', 'nav_action_check = shopping_zone_manager.nav_action_check:main', 'map_tour = shopping_zone_manager.map_tour:main', 'localization_check = shopping_zone_manager.localization_check:main']})
