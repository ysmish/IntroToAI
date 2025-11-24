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
_SIZE = None
_WALLS = None
_PLANTS_TARGETS = None
_ROBOTS_CAPACITIES = None
_PLANT_POSITIONS = None
_TAP_POSITIONS = None
_PLANT_INDEX = None
_TAP_INDEX = None

class WateringProblem(search.Problem):
    """This class implements a pressure plate problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        # Initialize module-level static values once to save time.
        # If they've already been set, this block is skipped.
        global _SIZE, _WALLS, _PLANTS_TARGETS, \
               _ROBOTS_CAPACITIES, _PLANT_POSITIONS, _TAP_POSITIONS

        if _SIZE is None:
            # Adjust the unpacking to match the exact structure of `initial` in your assignment.
            _SIZE = initial["Size"]
            _WALLS = frozenset(initial.get("Walls", set()))
            _PLANTS_TARGETS=dict(initial["Plants"])
            _ROBOTS_CAPACITIES = self._extract_robot_capacities(initial["Robots"])
            _PLANT_POSITIONS = tuple(sorted(frozenset(initial["Plants"].keys())))
            # keep taps in a fixed order (tuple) so we can index their remaining amounts consistently
            _TAP_POSITIONS = tuple(sorted(frozenset(initial["Taps"].keys())))

        # Initialize dynamic values.
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])

        initial_state = (robot_states, plant_states, tap_states)
        search.Problem.__init__(self, initial_state)

    @staticmethod
    #method to get the capacities of the robots from the initial data
    #is being separated because it is static data that does not change during the problem solving
    def _extract_robot_capacities(robots_data):
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        # frozenset of (robot_id, r = row, c = column, load = what's being currently carried)
        return frozenset(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        # store only the poured amounts in a fixed order (no repeated positions)
        # plant_states will be a tuple of ints; index i corresponds to sorted(_PLANT_POSITIONS)[i]
        # initial poured amount is 0 for every plant
        return tuple(0 for _ in _PLANT_POSITIONS)

    def _build_tap_states(self, taps_data):
        # store tap remaining amounts in a fixed order (tuple)
        # index i corresponds to sorted(_TAP_POSITIONS)[i]
        return tuple(taps_data.get(pos, 0) for pos in _TAP_POSITIONS)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_state, ...)]"""
        robot_states, plant_states, tap_states = state
        successors = []

        # iterate robots by reading entries from the frozenset
        for robot_entry in robot_states:
            robot_id, r, c, load = robot_entry

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states,
                                              robot_entry, robot_id, r, c, load)
            )

            successors.extend(
                self._get_load_successor(robot_states, plant_states, tap_states,
                                         robot_entry, robot_id, r, c, load)
            )

            successors.extend(
                self._get_pour_successor(robot_states, plant_states, tap_states,
                                         robot_entry, robot_id, r, c, load)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states,
                                 robot_entry, robot_id, r, c, load):
        """Generate movement successors for one robot."""
        successors = []

        # positions occupied by OTHER robots
        occupied = {
            (robot_r, robot_c)
            for (rid, robot_r, robot_c, _) in robot_states
            if rid != robot_id
        }

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc

            # bounds check
            if _SIZE is None:
                continue
            if not (0 <= nr < _SIZE[0] and 0 <= nc < _SIZE[1]):
                continue

            # wall check
            if (nr, nc) in _WALLS:
                continue

            # occupied check
            if (nr, nc) in occupied:
                continue

            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            new_state = (new_robot_states, plant_states, tap_states)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states,
                             robot_entry, robot_id, r, c, load):
        """Generate a LOAD successor if robot is on a tap with remaining water."""
        # tap positions must be initialized
        if _TAP_POSITIONS is None:
            return []

        # check if robot is standing on a tap
        if (r, c) not in _TAP_POSITIONS:
            return []

        # find index of this tap in the tap_states tuple
        try:
            tidx = _TAP_POSITIONS.index((r, c))
        except ValueError:
            return []

        remaining = tap_states[tidx]
        if remaining <= 0:
            return []

        # robot capacity lookup
        capacity = _ROBOTS_CAPACITIES.get(robot_id, 0)
        if load >= capacity:
            return []

        # update robot state (increment load)
        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        # update tap_states tuple (decrement remaining at tidx)
        new_tap_states = tuple(
            (remaining - 1) if i == tidx else amt
            for i, amt in enumerate(tap_states)
        )

        new_state = (new_robot_states, plant_states, new_tap_states)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states,
                             robot_entry, robot_id, r, c, load):
        """Generate a POUR successor if robot is on a plant and has water."""
        # plant positions must be initialized
        if _PLANT_POSITIONS is None:
            return []

        # robot must carry at least one unit
        if load <= 0:
            return []

        # check if robot is standing on a plant
        if (r, c) not in _PLANT_POSITIONS:
            return []

        # find index of this plant in the plant_states tuple
        try:
            pidx = _PLANT_POSITIONS.index((r, c))
        except ValueError:
            return []

        poured = plant_states[pidx]
        target = _PLANTS_TARGETS.get((r, c), 0)
        if poured >= target:
            return []

        # update robot state (decrement load)
        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        # update plant_states tuple (increment poured at pidx)
        new_plant_states = tuple(
            (poured + 1) if i == pidx else amt
            for i, amt in enumerate(plant_states)
        )

        new_state = (new_robot_states, new_plant_states, tap_states)
        return [((robot_id, "POUR"), new_state)]

    def _in_bounds(self, r, c):
        """Return True if (r,c) is inside the board defined by _SIZE."""
        if _SIZE is None:
            return False
        rows, cols = _SIZE
        return 0 <= r < rows and 0 <= c < cols

    def goal_test(self, state):
        """Return True iff every plant has received its target amount."""
        # state: (robot_states, plant_states, tap_states)
        _, plant_states, _ = state
        if _PLANT_POSITIONS is None or _PLANTS_TARGETS is None:
            return False
        for i, pos in enumerate(_PLANT_POSITIONS):
            poured = plant_states[i]
            target = _PLANTS_TARGETS.get(pos, 0)
            if poured < target:
                return False
        return True

    def h_astar(self, node):
        """A* admissible heuristic: 2*remaining_water - carried_by_robots (non-negative)."""
        state = node.state
        robot_states, _, _ = state
        remaining = self._remaining_water_need(state)
        carried = sum(load for (_, _, _, load) in robot_states)
        h = 2 * remaining - carried
        return max(0, h)

    def h_gbfs(self, node):
        """Greedy heuristic: remaining water need plus distance to closest unsatisfied plant."""
        state = node.state
        robot_states, plant_states, _ = state
        remaining = self._remaining_water_need(state)
        if remaining == 0:
            return 0

        if _PLANT_POSITIONS is None or _PLANTS_TARGETS is None:
            return remaining

        # unsatisfied plant positions
        unsatisfied = [
            pos
            for i, pos in enumerate(_PLANT_POSITIONS)
            if plant_states[i] < _PLANTS_TARGETS.get(pos, 0)
        ]

        if not unsatisfied:
            return remaining

        # robot positions
        robot_positions = [(rr, rc) for (_, rr, rc, _) in robot_states]
        if not robot_positions:
            return remaining

        min_distance = min(
            abs(pr - rr) + abs(pc - rc)
            for (rr, rc) in robot_positions
            for (pr, pc) in unsatisfied
        )

        return remaining + min_distance

    def _remaining_water_need(self, state):
        """Sum of remaining water required by all plants in the state."""
        _, plant_states, _ = state
        if _PLANT_POSITIONS is None or _PLANTS_TARGETS is None:
            return 0
        total = 0
        for i, pos in enumerate(_PLANT_POSITIONS):
            target = _PLANTS_TARGETS.get(pos, 0)
            poured = plant_states[i]
            total += max(0, target - poured)
        return total


def create_watering_problem(game):
    """ Create a pressure plate problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()