import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)

        # Initialize module-level static values once to save time.
        # If they've already been set, this block is skipped.
        global _SIZE, _WALLS, _PLANTS_TARGETS, \
               _ROBOTS_CAPACITIES, _PLANT_POSITIONS, _TAP_POSITIONS

        if _SIZE is None:
            # Adjust the unpacking to match the exact structure of `initial` in your assignment.
            _SIZE = initial_data["Size"]
            _WALLS = frozenset(initial_data.get("Walls", set()))
            _PLANTS_TARGETS=dict(initial_data["Plants"])
            _ROBOTS_CAPACITIES = self._extract_robot_capacities(initial_data["Robots"])
            _PLANT_POSITIONS = frozenset(initial_data["Plants"].keys())
            _TAP_POSITIONS = frozenset(initial_data["Taps"].keys())

            

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        utils.raiseNotDefined()

    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        utils.raiseNotDefined()

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        utils.raiseNotDefined()

    def h_gbfs(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        utils.raiseNotDefined()


def create_watering_problem(game):
    print("<<create_watering_problem")
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()