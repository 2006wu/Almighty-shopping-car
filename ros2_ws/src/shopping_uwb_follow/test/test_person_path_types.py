from shopping_uwb_follow.person_path import path_position


def test_integer_waypoints_yield_float_coordinates():
    path = dict(waypoints=[[1, 0], [1, 2]], speed=0.5, pauses=[3, 0])
    for elapsed in (0.0, 1.0, 4.0, 100.0):
        assert all(type(v) is float for v in path_position(path, elapsed))
    assert all(type(v) is float for v in path_position(dict(waypoints=[[0, 1]], speed=0), 5.0))
