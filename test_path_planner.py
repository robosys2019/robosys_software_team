"""This is a file to test the helper functions within the path_planner class"""

import unittest
from math import pi

from path_planner import PathPlanner

class PathPlannerTest(unittest.TestCase):
    def setUp(self):
        self.path_planner = PathPlanner()

    '''
    Function: 
    Tests:
    Calls:
    Notes:
    '''

    def testCalculate_angular_movement(self):
        # Test different movements for different start and end angles in pi/4 incerements

        # if you start at 0 and have increasing targets, these are the correct movements. You can shift back one each time you increase the starting angle
        correct_movements = [0, -pi/4, -pi/2, -pi*(3/4), -pi, pi*(3/4), pi/2, pi/4]

        for i in range(0,8):
            # Test targets from 0 to 2pi in pi/4 increments
            target = pi * i/4
            print("TARGET PI * {}/4".format(i))
            for j in range(0,8):
                # Test starting place from 0 to 2pi for each target
                current = pi * (j/4)
                movement = self.path_planner.calculate_angular_movement(current, target)
                self.assertEqual(correct_movements[j-i], movement)
    
if __name__ == '__main__':
    unittest.main()