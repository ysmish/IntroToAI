import ext_plant
from collections import deque
import math
import bisect
import sys

# Update with your ID
id = ["216764803"]

# Directions mapping for BFS
_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

# ==============================================================================
# 1. COPIED UTILS & SEARCH CLASSES (Dependencies for A*)
# ==============================================================================

infinity = 1.0e400

def update(x, **entries):
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x

def memoize(fn, slot=None):
    if slot:
        def memoized_fn(obj, *args):
            if hasattr(obj, slot):
                return getattr(obj, slot)
            else:
                val = fn(obj, *args)
                setattr(obj, slot, val)
                return val
    else:
        def memoized_fn(*args):
            if not memoized_fn.cache.has_key(args):
                memoized_fn.cache[args] = fn(*args)
            return memoized_fn.cache[args]
        memoized_fn.cache = {}
    return memoized_fn

class Queue:
    def __init__(self):
        raise NotImplementedError

    def extend(self, items):
        for item in items: self.append(item)

class PriorityQueue(Queue):
    def __init__(self, order=min, f=lambda x: x):
        update(self, A=[], order=order, f=f)

    def append(self, item):
        bisect.insort(self.A, (self.f(item), item))

    def __len__(self):
        return len(self.A)

    def pop(self):
        if self.order == min:
            return self.A.pop(0)[1]
        else:
            return self.A.pop()[1]

class Node:
    """A node in a search tree."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        update(self, state=state, parent=parent, action=action,
               path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def path(self):
        x, result = self, [self]
        while x.parent:
            result.append(x.parent)
            x = x.parent
        return result

    def expand(self, problem):
        return [Node(next, self, act,
                     problem.path_cost(self.path_cost, self.state, act, next))
                for (act, next) in problem.successor(self.state)]

    # --- Added Comparison Methods (Crucial for PriorityQueue) ---
    def __eq__(self, other):
        return (self.f == other.f)

    def __ne__(self, other):
        return not (self == other)

    def __lt__(self, other):
        return (self.f < other.f)

    def __gt__(self, other):
        return (self.f > other.f)

    def __le__(self, other):
        return (self < other) or (self == other)

    def __ge__(self, other):
        return (self > other) or (self == other)

class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def successor(self, state):
        raise NotImplementedError

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self):
        raise NotImplementedError

# --- Search Algorithms ---

def graph_search(problem, fringe):
    closed = {}
    expanded = 0
    fringe.append(Node(problem.initial))
    while fringe:
        node = fringe.pop()
        if problem.goal_test(node.state):
            return node, expanded
        if node.state not in closed:
            closed[node.state] = True
            fringe.extend(node.expand(problem))
            expanded += 1
    return None

def best_first_graph_search(problem, f):
    f = memoize(f, 'f')
    return graph_search(problem, PriorityQueue(min, f))

def astar_search(problem, h=None):
    h = h or problem.h
    def f(n):
        return max(getattr(n, 'f', -infinity), n.path_cost + h(n))
    return best_first_graph_search(problem, f)


# ==============================================================================
# 2. WATERING PROBLEM CLASS (From ex1.py)
# ==============================================================================

class WateringProblem(Problem):
    """
    This class implements the Watering Problem for AI Search.
    It solves the problem of moving robots to taps to fill up and to plants to pour water.
    """

    def __init__(self, initial):
        # --- 1. Parse Initial State ---
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        # Sort positions for deterministic behavior
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        # --- 2. Pre-Calculate BFS Maps ---
        self.plant_bfs_maps = {}
        for p_pos in self.plant_positions:
            self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
        
        self.tap_bfs_maps = {}
        for t_pos in self.tap_positions:
            self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- 3. OPTIMIZATION: Highway Pruning (Single Robot) ---
        robot_starts = set((r, c) for (r, c, _, _) in initial["Robots"].values())
        num_robots = len(initial["Robots"])

        if num_robots == 1:
            start_pos = list(robot_starts)[0]
            start_bfs_map = self._bfs_map(start_pos)
            pois = set(self.plant_positions) | set(self.tap_positions) | {start_pos}
            
            def get_dist_map(p):
                if p == start_pos: return start_bfs_map
                if p in self.plant_bfs_maps: return self.plant_bfs_maps[p]
                if p in self.tap_bfs_maps: return self.tap_bfs_maps[p]
                return None

            valid_highway_cells = set()
            pairs = []
            for target in pois:
                if start_pos != target: pairs.append((start_pos, target))
            
            taps = list(self.tap_positions)
            plants = list(self.plant_positions)
            for t in taps:
                for p in plants:
                    pairs.append((t, p)); pairs.append((p, t))
            for p1 in plants:
                for p2 in plants:
                    if p1 != p2: pairs.append((p1, p2))

            for src, dst in pairs:
                d_map_src = get_dist_map(src)
                d_map_dst = get_dist_map(dst)
                if not d_map_src or dst not in d_map_src: continue
                
                shortest_len = d_map_src[dst]
                queue = [src]
                visited_local = {src}
                valid_highway_cells.add(src)
                
                idx = 0
                while idx < len(queue):
                    curr = queue[idx]
                    idx += 1
                    if curr == dst: continue 
                    
                    for dr, dc in _MOVES.values():
                        nr, nc = curr[0] + dr, curr[1] + dc
                        if (nr, nc) in visited_local: continue
                        if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
                        
                        d_n_src = d_map_src.get((nr, nc))
                        d_n_dst = d_map_dst.get((nr, nc))
                        
                        if d_n_src is not None and d_n_dst is not None:
                            if d_n_src + d_n_dst == shortest_len:
                                valid_highway_cells.add((nr, nc))
                                visited_local.add((nr, nc))
                                queue.append((nr, nc))

            if valid_highway_cells: 
                new_walls = set(self.walls)
                for r in range(self.size[0]):
                    for c in range(self.size[1]):
                        if (r, c) not in self.walls and (r, c) not in valid_highway_cells:
                            new_walls.add((r, c))
                self.walls = frozenset(new_walls)

            # Re-calculate maps
            self.plant_bfs_maps = {}
            for p_pos in self.plant_positions:
                self.plant_bfs_maps[p_pos] = self._bfs_map(p_pos)
            self.tap_bfs_maps = {}
            for t_pos in self.tap_positions:
                self.tap_bfs_maps[t_pos] = self._bfs_map(t_pos)

        # --- 4. Initialize Dynamic State ---
        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        total_remaining = sum(initial["Plants"].values())

        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        Problem.__init__(self, initial_state)

    def _bfs_map(self, start_pos):
        queue = [(start_pos, 0)]
        distances = {start_pos: 0}
        visited = {start_pos}
        idx = 0
        while idx < len(queue):
            (r, c), dist = queue[idx]
            idx += 1
            for dr, dc in _MOVES.values():
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size[0] and 0 <= nc < self.size[1]:
                    if (nr, nc) not in self.walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        new_dist = dist + 1
                        distances[(nr, nc)] = new_dist
                        queue.append(((nr, nc), new_dist))
        return distances

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        return frozenset(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []
        current_total_load = sum(l for (_, _, _, l) in robot_states)

        for robot_entry in sorted(robot_states):
            robot_id, r, c, load = robot_entry

            occupied = {
                (or_r, or_c)
                for (oid, or_r, or_c, _) in robot_states
                if oid != robot_id
            }
            
            is_blocking = False
            for dr, dc in _MOVES.values():
                if (r + dr, c + dc) in occupied:
                    is_blocking = True; break

            possible_actions = []

            # Try POUR
            if load > 0 and (r, c) in self.plant_positions:
                 possible_actions.extend(
                    self._get_pour_successor(robot_states, plant_states, tap_states, total_remaining,
                                             robot_entry, robot_id, r, c, load)
                 )

            # Try LOAD
            if load < self.robots_capacities[robot_id] and (r, c) in self.tap_positions:
                if total_remaining > current_total_load:
                    possible_actions.extend(
                        self._get_load_successor(robot_states, plant_states, tap_states, total_remaining,
                                                 robot_entry, robot_id, r, c, load)
                    )

            if possible_actions:
                successors.extend(possible_actions)
                if not is_blocking:
                    continue 

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states, total_remaining,
                                              robot_entry, robot_id, r, c, load, occupied)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states, total_remaining,
                                 robot_entry, robot_id, r, c, load, occupied):
        successors = []
        capacity = self.robots_capacities[robot_id]
        
        if load == 0:
            target_maps = list(self.tap_bfs_maps.values())
        elif load == capacity:
            target_maps = list(self.plant_bfs_maps.values())
        else:
            target_maps = list(self.tap_bfs_maps.values()) + list(self.plant_bfs_maps.values())

        valid_candidates = []
        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            if (nr, nc) in occupied: continue
            valid_candidates.append((action_name, nr, nc))

        improving_candidates = []
        for action_name, nr, nc in valid_candidates:
            is_improving = False
            for bfs_map in target_maps:
                curr_dist = bfs_map.get((r, c))
                next_dist = bfs_map.get((nr, nc))
                
                if curr_dist is not None and next_dist is not None:
                    if next_dist == curr_dist - 1:
                        is_improving = True; break
            
            if is_improving:
                improving_candidates.append((action_name, nr, nc))

        final_candidates = improving_candidates if improving_candidates else valid_candidates

        for action_name, nr, nc in final_candidates:
            old_robot_entry = robot_entry
            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {old_robot_entry}) | {new_robot_entry}
            
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            
            action_str = f"{action_name}{{{robot_id}}}"
            successors.append((action_str, new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        if (r, c) not in self.tap_positions: return []
        try:
            tidx = self.tap_positions.index((r, c))
        except ValueError: return []

        remaining_tap = tap_states[tidx]
        if remaining_tap <= 0: return []

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
        
        action_str = f"LOAD{{{robot_id}}}"
        return [(action_str, new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states, total_remaining,
                             robot_entry, robot_id, r, c, load):
        if load <= 0: return []
        if (r, c) not in self.plant_positions: return []
        try:
            pidx = self.plant_positions.index((r, c))
        except ValueError: return []
        
        poured = plant_states[pidx]
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
        
        action_str = f"POUR{{{robot_id}}}"
        return [(action_str, new_state)]

    def goal_test(self, state):
        return state[3] == 0

    def h_astar(self, node):
        state = node.state
        robot_states, plant_states, _, total_remaining = state
        
        robot_positions = [(r, c) for (_, r, c, _) in sorted(robot_states)]
        unsatisfied_plants = []
        for i, pos in enumerate(self.plant_positions):
            if plant_states[i] < self.plants_targets.get(pos, 0):
                unsatisfied_plants.append(pos)
        
        carried = sum(load for (_, _, _, load) in robot_states)
        interaction_cost = (2 * total_remaining) - carried
        
        if not unsatisfied_plants:
            return max(0, interaction_cost)
            
        num_robots = len(robot_positions)
        total_vertices = num_robots + len(unsatisfied_plants)
        
        visited = [False] * total_vertices
        min_dists = [float('inf')] * total_vertices
        min_dists[0] = 0
        mst_weight = 0
        
        for _ in range(total_vertices):
            u = -1
            min_val = float('inf')
            for i in range(total_vertices):
                if not visited[i] and min_dists[i] < min_val:
                    min_val = min_dists[i]
                    u = i
            
            if u == -1 or min_val == float('inf'): break
                
            visited[u] = True
            mst_weight += min_val
            
            pos_u = None
            is_robot_u = (u < num_robots)
            if is_robot_u:
                pos_u = robot_positions[u]
            else:
                pos_u = unsatisfied_plants[u - num_robots]

            for v in range(total_vertices):
                if not visited[v]:
                    weight = float('inf')
                    is_robot_v = (v < num_robots)
                    
                    if is_robot_u and is_robot_v:
                        weight = 0 
                    else:
                        pos_v = robot_positions[v] if is_robot_v else unsatisfied_plants[v - num_robots]
                        raw_dist = float('inf')
                        
                        if not is_robot_u:
                            raw_dist = self.plant_bfs_maps[pos_u].get(pos_v, float('inf'))
                        elif not is_robot_v:
                            raw_dist = self.plant_bfs_maps[pos_v].get(pos_u, float('inf'))
                            
                        if raw_dist != float('inf'):
                            weight = raw_dist / 1.5
                            
                    if weight < min_dists[v]:
                        min_dists[v] = weight
                        
        return mst_weight + max(0, interaction_cost)
    
    # Mapping 'h' to 'h_astar' for the search algorithm
    h = h_astar


class Controller:
    """Controller for the stochastic Plant Watering problem using BFS Gradient Maps."""

    def __init__(self, game: ext_plant.Game):
        """
        Constructor. Parses input and pre-calculates BFS distance maps
        to create a 'gradient' that guides robots from ANY cell.
        """
        # This part is for the specific ex2 task (not strictly needed for the a_star request but good to keep)
        self.game = game

    def a_star(self, state):
        """
        Returns an optimal moves plan using the exact A* logic from ex1.py.
        """
        # 1. Initialize the problem with the given state
        problem = WateringProblem(state)
        
        # 2. Run A* search
        # Note: In the provided search.py, graph_search returns (node, expanded).
        # astar_search calls best_first_graph_search which calls graph_search.
        result = astar_search(problem)
        
        # Handle the result. If no solution found, result might be None.
        if result is None:
            return []
            
        # The provided search.py returns a tuple (node, expanded)
        goal_node = result[0]
        
        # 3. Reconstruct the path of actions
        actions = []
        curr = goal_node
        while curr.parent is not None:
            actions.append(curr.action)
            curr = curr.parent
            
        # Reverse to get Start -> Goal
        return actions[::-1]

    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        raise ValueError("Fill the function")