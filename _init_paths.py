import os.path as osp
import sys


def init_paths():
    def add_path(path):
        if path not in sys.path:
            sys.path.insert(0, path)

    this_dir = osp.dirname(__file__)

    # Add libs to PYTHONPATH
    # Add failure types libraries
    lib_path_failures = osp.join(this_dir, 'Drone_flight\\Failure_injection\\FailureTypes')
    add_path(lib_path_failures)

    # Add Navigation Algorithms libraries
    lib_path_nav = osp.join(this_dir, 'Drone_grid_navigation/PythonRobotics')
    add_path(lib_path_nav)
