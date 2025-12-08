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
    """This class implements a plant watering problem"""

    def __init__(self, initial):
        """ Constructor only needs the initial state. """
        # 1. Store "Static" map data in self. 
        # These are calculated ONCE per problem instance and valid for the whole search.
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Pre-calculating sorted tuples for consistent indexing
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # 2. Initialize dynamic values.
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        
        # Calculate total water needed globally (Sum of all targets)
        total_remaining = sum(initial["Plants"].values())

        # State is: (robot_states, plant_states, tap_states, total_remaining)
        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        search.Problem.__init__(self, initial_state)

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        # frozenset of (robot_id, r, c, load)
        return frozenset(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        # store only the poured amounts in a fixed order based on self.plant_positions
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        # store tap remaining amounts in a fixed order based on self.tap_positions
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_state, ...)]"""
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []

        for robot_entry in robot_states:
            robot_id, r, c, load = robot_entry

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load)
            )

            successors.extend(
                self._get_load_successor(robot_states, plant_states, tap_states, total_remaining,
                                         robot_entry, robot_id, r, c, load)
            )

            successors.extend(
                self._get_pour_successor(robot_states, plant_states, tap_states, total_remaining,
                                         robot_entry, robot_id, r, c, load)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states, total_remaining,
                                 robot_entry, robot_id, r, c, load):
        """Generate movement successors for one robot."""
        successors = []

        occupied = {
            (robot_r, robot_c)
            for (rid, robot_r, robot_c, _) in robot_states
            if rid != robot_id
        }

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc

            # Bounds check using self.size
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            
            # Wall check using self.walls
            if (nr, nc) in self.walls: continue
            
            # Robot collision check
            if (nr, nc) in occupied: continue

            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            
            # Movement does not change total_remaining
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        """Generate a LOAD successor."""
        # Use self.tap_positions
        if (r, c) not in self.tap_positions: return []

        try:
            tidx = self.tap_positions.index((r, c))
        except ValueError: return []

        remaining_tap = tap_states[tidx]
        if remaining_tap <= 0: return []

        # Use self.robots_capacities
        capacity = self.robots_capacities.get(robot_id, 0)
        if load >= capacity: return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        new_tap_states = tuple(
            (remaining_tap - 1) if i == tidx else amt
            for i, amt in enumerate(tap_states)
        )

        new_state = (new_robot_states, plant_states, new_tap_states, total_remaining)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        """Generate a POUR successor."""
        if load <= 0: return []
        
        # Use self.plant_positions
        if (r, c) not in self.plant_positions: return []

        try:
            pidx = self.plant_positions.index((r, c))
        except ValueError: return []

        poured = plant_states[pidx]
        
        # Use self.plants_targets
        target = self.plants_targets.get((r, c), 0)
        
        if poured >= target: return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        new_plant_states = tuple(
            (poured + 1) if i == pidx else amt
            for i, amt in enumerate(plant_states)
        )

        new_total_remaining = total_remaining - 1
        
        new_state = (new_robot_states, new_plant_states, tap_states, new_total_remaining)
        return [((robot_id, "POUR"), new_state)]

    def goal_test(self, state):
        """Return True iff every plant has received its target amount."""
        return state[3] == 0

    def h_astar(self, node):
        """A* admissible heuristic: 2*remaining_water - carried_by_robots (non-negative)."""
        state = node.state
        robot_states, _, _, total_remaining = state
        
        remaining = total_remaining
        carried = sum(load for (_, _, _, load) in robot_states)
        
        # Admissible logic: we must at least load and pour every remaining unit,
        # minus what we are already carrying.
        h = 2 * remaining - carried
        return max(0, h)

    def h_gbfs(self, node):
        """Greedy heuristic."""
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        remaining = total_remaining
        
        if remaining == 0:
            return 0

        # Calculate distance to unsatisfied plants
        unsatisfied = [
            pos
            for i, pos in enumerate(self.plant_positions)
            if plant_states[i] < self.plants_targets.get(pos, 0)
        ]

        if not unsatisfied:
            return remaining

        robot_positions = [(rr, rc) for (_, rr, rc, _) in robot_states]
        if not robot_positions:
            return remaining

        # Find minimum Manhattan distance from any robot to any unsatisfied plant
        min_distance = min(
            abs(pr - rr) + abs(pc - rc)
            for (rr, rc) in robot_positions
            for (pr, pc) in unsatisfied
        )

        return remaining + min_distance


def create_watering_problem(game):
    return WateringProblem(game)


if __name__ == '__main__':
    ex1_check.main()