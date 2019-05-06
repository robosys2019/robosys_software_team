from math import pi
from path_planner import PathPlanner, Node
from map_maker import MapMaker

class RoverMovementTest():
    def __init__(self):
        self.path_planner = PathPlanner()

        # will need a map:
        map_maker = MapMaker()
        map_maker.set_map()
        map_maker.calibrate_map()
        lowres_map = map_maker.get_lowres_map()
        self.path_planner.set_map(lowres_map)

    def run_width_test(self, test=False):
        # create path along bottom of map (left to right)
        # assume rover placed parallel to x axis, moving positive (angle 0)
        path = [[0,0], [1,0],[2,0],[3,0], [4,0], [5,0]]
        for i in range(0,len(path)):
            path[i] = self.path_planner.pos_from_coordinates(path[i])
        self.path_planner.path = path
        self.path_planner.start_node = Node(pos = self.path_planner.path[0])
        self.path_planner.end_node = Node(pos = self.path_planner.path[-1])

        current_coord = self.path_planner.coordinates_from_pos(self.path_planner.start_node.pos)
        current_angle = 0

        while len(self.path_planner.path) > 1:
            if test:
                message, current_coord, current_angle = self.path_planner.get_action(current_coord, current_angle, test=True)
                print(message + " STEPS LEFT:{}".format(len(self.path_planner.path)))
            else:
                self.path_planner.get_action(current_coord, current_angle, test=False)
        return

    def one_step(self, test=False):
        path = [[0,0], [1,0]]
        for i in range(0,len(path)):
            path[i] = self.path_planner.pos_from_coordinates(path[i])
        self.path_planner.path = path
        self.path_planner.start_node = Node(pos = self.path_planner.path[0])
        self.path_planner.end_node = Node(pos = self.path_planner.path[-1])

        current_coord = self.path_planner.coordinates_from_pos(self.path_planner.start_node.pos)
        current_angle = 0

        while len(self.path_planner.path) > 1:
            if test:
                message, current_coord, current_angle = self.path_planner.get_action(current_coord, current_angle, test=True)
                print(message + " STEPS LEFT:{}".format(len(self.path_planner.path)))
            else:
                self.path_planner.get_action(current_coord, current_angle, test=False)
        return

if __name__ == '__main__':
    rover_movement_test = RoverMovementTest()

    # test cases:
    # rover_movement_test.run_width_test()
    rover_movement_test.one_step()