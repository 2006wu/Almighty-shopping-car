from glob import glob
from setuptools import setup

package_name = 'shopping_mcu_bridge'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml'])] +
                 [('share/'+package_name+'/'+d, glob(d+'/*')) for d in ['config', 'launch']],
      install_requires=['setuptools', 'PyYAML'], tests_require=['pytest'], zip_safe=True,
      maintainer='2006wu', maintainer_email='2006wu@users.noreply.github.com',
      description='Simulation-first MCU communication contract and transports', license='Apache-2.0',
      entry_points={'console_scripts': [
          'sim_mcu_firmware = shopping_mcu_bridge.nodes:firmware_main',
          'serial_bridge = shopping_mcu_bridge.nodes:serial_main',
          'mcu_link_monitor = shopping_mcu_bridge.monitor_node:main',
          'agent_check = shopping_mcu_bridge.agent_check:main',
          'comm_check = shopping_mcu_bridge.comm_check:main']})
