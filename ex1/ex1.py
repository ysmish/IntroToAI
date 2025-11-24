import search
import utils

id = ["No numbers - I'm special!"]

_MOVE_DELTAS = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

# Use a helper function for O(N) lookup from a frozenset
def _find_entity_state(frozenset_data, key):
    # Returns (entity_key, value) or None. This remains O(N) but centralises the pattern.
    return next(((entity_key, value) for (entity_key, value) in frozenset_data if entity_key == key), None)

class StaticInfo:
    """Holds all static, shared information for a WateringProblem instance."""
    def __init__(self, size, walls, plant_targets, robot_capacities,
                 plant_positions, tap_positions):
        self.size = size
        self.walls = walls
        self.plant_targets = plant_targets
        self.robot_capacities = robot_capacities
        # plant_positions and tap_positions are frozensets of coordinate tuples
        self.plant_positions = plant_positions
        self.tap_positions = tap_positions

class WateringProblem(search.Problem):
    """Multi-robot watering problem with taps, plants, and walls.
    State format: (static, robot_states, plant_states, tap_states)
    where:
      - static is a shared StaticInfo instance
      - robot_states is a frozenset of (robot_id, r, c, load)
      - plant_states is a frozenset of ((r, c), poured)
      - tap_states is a frozenset of ((r, c), remaining)
    """

    def __init__(self, initial_data):
        # Build static data once and store in one object
        size = initial_data["Size"]
        walls = frozenset(initial_data.get("Walls", set()))
        plant_targets = dict(initial_data["Plants"])
        robot_capacities = self._extract_robot_capacities(initial_data["Robots"])

        # dynamic states as frozensets of items (hashable)
        robot_states = self._build_robot_states(initial_data["Robots"])
        plant_states = self._build_plant_states(initial_data["Plants"])
        tap_states = self._build_tap_states(initial_data["Taps"])

        # static quick-lookup sets for positions (immutable)
        plant_positions = frozenset(initial_data["Plants"].keys())
        tap_positions = frozenset(initial_data["Taps"].keys())

        self._static = StaticInfo(size, walls, plant_targets, robot_capacities,
                                  plant_positions, tap_positions)

        # initial state includes the shared static object
        initial_state = (self._static, robot_states, plant_states, tap_states)
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
        # frozenset of ((r, c), poured)
        return frozenset(
            ((r, c), 0)
            for (r, c) in sorted(plants_data.keys())
        )

    def _build_tap_states(self, taps_data):
        # frozenset of ((r, c), remaining)
        return frozenset(
            ((r, c), remaining)
            for (r, c), remaining in sorted(taps_data.items())
        )

    def successor(self, state):
        static, robot_states, plant_states, tap_states = state
        successors = []

        # iterate robots by reading entries from the frozenset
        for robot_entry in robot_states:
            robot_id, r, c, load = robot_entry

            successors.extend(
                self._get_movement_successors(static, robot_states, plant_states, tap_states,
                                              robot_entry, robot_id, r, c, load)
            )

            successors.extend(
                self._get_load_successor(static, robot_states, plant_states, tap_states,
                                         robot_entry, robot_id, r, c, load)
            )
            successors.extend(
                self._get_pour_successor(static, robot_states, plant_states, tap_states,
                                         robot_entry, robot_id, r, c, load)
            )

        return successors

    def _get_movement_successors(self, static, robot_states, plant_states, tap_states,
                                 robot_entry, robot_id, r, c, load):
        successors = []
        occupied = {
            (robot_r, robot_c)
            for (rid, robot_r, robot_c, _) in robot_states
            if rid != robot_id
        }

        for action_name, (dr, dc) in _MOVE_DELTAS.items():
            nr, nc = r + dr, c + dc
            if not self._in_bounds(nr, nc):
                continue
            if (nr, nc) in static.walls:
                continue
            if (nr, nc) in occupied:
                continue

            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            new_state = (static, new_robot_states, plant_states, tap_states)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, static, robot_states, plant_states, tap_states,
                            robot_entry, robot_id, r, c, load):
        # find tap entry at robot location (if any)
        if (r, c) not in static.tap_positions:
            return []

        tap_entry = _find_entity_state(tap_states, (r, c))
        if tap_entry is None:
            return []

        _, remaining = tap_entry
        if remaining <= 0:
            return []

        capacity = static.robot_capacities[robot_id]
        if load >= capacity:
            return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        new_tap_entry = (tap_entry[0], remaining - 1)
        new_tap_states = (tap_states - {tap_entry}) | {new_tap_entry}

        new_state = (static, new_robot_states, plant_states, new_tap_states)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, static, robot_states, plant_states, tap_states,
                            robot_entry, robot_id, r, c, load):
        if load <= 0:
            return []

        if (r, c) not in static.plant_positions:
            return []

        pos = (r, c)
        plant_entry = _find_entity_state(plant_states, pos)
        if plant_entry is None:
            return []

        pos, poured = plant_entry
        target = static.plant_targets[pos]
        if poured >= target:
            return []

        old_robot_entry = robot_entry
        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}

        new_plant_entry = (pos, poured + 1)
        new_plant_states = (plant_states - {plant_entry}) | {new_plant_entry}

        new_state = (static, new_robot_states, new_plant_states, tap_states)
        return [((robot_id, "POUR"), new_state)]

    def _in_bounds(self, r, c):
        rows, cols = self._static.size
        return 0 <= r < rows and 0 <= c < cols

    def goal_test(self, state):
        static, _, plant_states, _ = state
        for (r, c), poured in plant_states:
            if poured < static.plant_targets[(r, c)]:
                return False
        return True

    def h_astar(self, node):
        static, robot_states, plant_states, _ = node.state
        remaining = self._remaining_water_need(node.state)
        carried = sum(load for (_, _, _, load) in robot_states)
        return 2*remaining - carried

    def h_gbfs(self, node):
        static, robot_states, plant_states, _ = node.state
        remaining = self._remaining_water_need(node.state)
        if remaining == 0:
            return 0

        unsatisfied_plants = [
            pos
            for (pos, poured) in plant_states
            if poured < static.plant_targets[pos]
        ]

        if not unsatisfied_plants:
            return remaining

        min_distance = min(
            abs(pr - rr) + abs(pc - rc)
            for (rr, rc) in ((robot_r, robot_c) for (_, robot_r, robot_c, _) in robot_states)
            for (pr, pc) in unsatisfied_plants
        )

        return remaining + min_distance

    def _remaining_water_need(self, state):
        static, _, plant_states, _ = state
        return sum(
            max(0, static.plant_targets[pos] - poured)
            for (pos, poured) in plant_states
        )


def create_watering_problem(game):
    """Factory method retained for checker compatibility."""
    return WateringProblem(game)


if __name__ == '__main__':
    import ex1_check

    ex1_check.main()