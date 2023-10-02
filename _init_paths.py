#!/usr/bin/env python
"""
Provides the function that adds some folders to the path.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

# Imports
import os.path as osp
import sys


def init_paths():
    """
    Function that adds some hard-coded folders of the repository to the path before the main code is run
    :return: None
    """
    def add_path(path):
        """
        Function that adds any file path to the system path
        :param path: location of the folder to add to the system path
        :return: None
        """
        if path not in sys.path:
            sys.path.insert(0, path)

    this_dir = osp.dirname(__file__)

    # Add libs to PYTHONPATH
    # Add failure types libraries
    lib_path_failures = osp.join(this_dir, 'Drone_flight\\Failure_injection\\FailureTypes')
    add_path(lib_path_failures)

    # Add Navigation Algorithms libraries
    lib_path_nav = osp.join(this_dir, 'Drone_grid_navigation\\PythonRobotics')
    add_path(lib_path_nav)
