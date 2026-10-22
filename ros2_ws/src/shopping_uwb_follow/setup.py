from glob import glob
from setuptools import setup
package_name = 'shopping_uwb_follow'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])] +
                 [('share/' + package_name + '/' + d, glob(d + '/*')) for d in ['config', 'launch']],
      install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
      maintainer='2006wu', maintainer_email='2006wu@users.noreply.github.com',
      description='Simulation shopping robot logic', license='Apache-2.0',
      entry_points={'console_scripts': ['uwb_eval_node = shopping_uwb_follow.uwb_eval_node:main', 'uwb_sim_node = shopping_uwb_follow.uwb_sim_node:main', 'uwb_estimator_node = shopping_uwb_follow.uwb_estimator_node:main', 'follow_controller_node = shopping_uwb_follow.follow_controller_node:main', 'person_path_node = shopping_uwb_follow.person_path_node:main']})
