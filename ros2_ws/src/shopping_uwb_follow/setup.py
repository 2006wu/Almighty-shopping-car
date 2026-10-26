import os
from glob import glob
from setuptools import setup
package_name = 'shopping_uwb_follow'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])] +
                 [('share/' + package_name + '/' + d, [f for f in glob(d+'/*') if os.path.isfile(f)]) for d in ['config', 'launch']],
      install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
      maintainer='2006wu', maintainer_email='2006wu@users.noreply.github.com',
      description='Simulation shopping robot logic', license='Apache-2.0',
      entry_points={'console_scripts': ['person_sync_check = shopping_uwb_follow.evaluation_node:person_sync_main', 'follow_eval = shopping_uwb_follow.evaluation_node:follow_eval_main', 'wheel_odom_noise = shopping_uwb_follow.wheel_odom_noise_node:main', 'odom_drift_monitor = shopping_uwb_follow.odom_drift_monitor_node:main', 'uwb_eval_node = shopping_uwb_follow.uwb_eval_node:main', 'uwb_sim_node = shopping_uwb_follow.uwb_sim_node:main', 'uwb_estimator_node = shopping_uwb_follow.uwb_estimator_node:main', 'follow_controller_node = shopping_uwb_follow.follow_controller_node:main', 'person_path_node = shopping_uwb_follow.person_path_node:main']})
