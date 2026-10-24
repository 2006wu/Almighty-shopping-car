"""ROS-independent stack choice and apt environment isolation."""
FORK_PACKAGES = frozenset({'custom_controller', 'custom_nav_to_pose'})


def select_stack(requested, available):
    if requested not in ('auto', 'apt', 'fork'):
        raise ValueError('nav2_stack must be auto, apt or fork')
    ready = FORK_PACKAGES <= set(available)
    if requested == 'fork' and not ready:
        raise ValueError('Nav2 fork unavailable / 尚未建置: run ./scripts/dev.sh nav2-build')
    return ('fork' if ready else 'apt') if requested == 'auto' else requested


def apt_environment(environment):
    """Remove fork search paths before sourcing apt and the application's local setup."""
    result = dict(environment)
    for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                'LD_LIBRARY_PATH', 'PYTHONPATH', 'PATH'):
        if key in result:
            result[key] = ':'.join(p for p in result[key].split(':')
                                   if p and not p.startswith('/nav2_ws/'))
    result['SHOP_NAV2_APT_ONLY'] = '1'
    return result
