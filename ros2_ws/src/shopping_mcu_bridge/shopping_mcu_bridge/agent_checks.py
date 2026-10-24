"""ROS-free graph and telemetry predicates for isolated XRCE simulation checks."""
import math


def isolated_graph(nodes, publishers, readers, clock_publishers, marker):
    required = ('micro_ros_sim_client', 'mcu_link_monitor', 'cmd_vel_mux', 'cmd_vel_watchdog')
    if not marker or clock_publishers or any(nodes.count(n) != 1 for n in required):
        return False
    if any('gazebo' in n.lower() or n in ('sim_mcu_firmware', 'serial_bridge') for n in nodes):
        return False
    expected = {'/cmd_vel': ['cmd_vel_watchdog'], '/pc/heartbeat': ['mcu_link_monitor'],
                **{topic: ['micro_ros_sim_client'] for topic in
                   ('/mcu/heartbeat', '/mcu/wheel_state', '/mcu/status', '/mcu/sim/xrce_active')}}
    return all(publishers.get(t) == owners for t, owners in expected.items()) and readers == ['micro_ros_sim_client']


def zero_window(samples, start, minimum=5):
    recent = [m for t, m in samples if t >= start]
    return len(recent) >= minimum and all(
        len(m.velocity) == 4 and all(math.isfinite(v) and abs(v) < 1e-6 for v in m.velocity)
        for m in recent)


def advancing_counter(samples):
    counters = [m.data for _, m in samples]
    return len(counters) >= 10 and all(0 < ((b-a) & 0xffffffff) < 0x80000000
                                     for a, b in zip(counters, counters[1:]))
