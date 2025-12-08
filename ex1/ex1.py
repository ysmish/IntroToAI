import search
import math

id = ["No numbers - I'm special!"]

_MOVES = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}

class WateringProblem(search.Problem):
    """This class implements a Multi-Tap Plant Watering problem"""

    def __init__(self, initial):
        """ Constructor initializes the problem and pre-calculates maps. """
        
        self.size = initial["Size"]
        self.walls = frozenset(initial.get("Walls", set()))
        self.plants_targets = dict(initial["Plants"])
        self.robots_capacities = self._extract_robot_capacities(initial["Robots"])
        
        self.plant_positions = tuple(sorted(frozenset(initial["Plants"].keys())))
        self.tap_positions = tuple(sorted(frozenset(initial["Taps"].keys())))

        self.plant_distances = {}
        self.tap_distances = {}
        
        # Pre-compute Tunnel Map for smart pruning
        self.tunnel_map = self._build_tunnel_map()

        # --- OPTIMIZATION: Single Robot Pruning ---
        if len(initial["Robots"]) == 1:
            robot_start = list(initial["Robots"].values())[0][0:2]
            pois = set(self.plant_positions) | set(self.tap_positions) | {robot_start}
            
            poi_distances = {}
            for p in pois:
                poi_distances[p] = self._bfs_map(p, self.walls, self.size)

            keep_cells = set()
            poi_list = list(pois)
            candidate_cells = set().union(*[d.keys() for d in poi_distances.values()])

            for i in range(len(poi_list)):
                u = poi_list[i]
                for j in range(i + 1, len(poi_list)):
                    v = poi_list[j]
                    dist_uv = poi_distances[u].get(v)
                    if dist_uv is None: continue

                    for cell in candidate_cells:
                        if cell in keep_cells: continue
                        d_uc = poi_distances[u].get(cell)
                        d_cv = poi_distances[v].get(cell)
                        if d_uc is not None and d_cv is not None:
                            if d_uc + d_cv == dist_uv:
                                keep_cells.add(cell)
            
            new_walls = set(self.walls)
            for r in range(self.size[0]):
                for c in range(self.size[1]):
                    if (r, c) not in keep_cells:
                        new_walls.add((r, c))
            self.walls = frozenset(new_walls)

            for p in self.plant_positions:
                self.plant_distances[p] = poi_distances[p]
            for t in self.tap_positions:
                self.tap_distances[t] = poi_distances[t]

        else:
            for plant_pos in self.plant_positions:
                self.plant_distances[plant_pos] = self._bfs_map(plant_pos, self.walls, self.size)
            for tap_pos in self.tap_positions:
                self.tap_distances[tap_pos] = self._bfs_map(tap_pos, self.walls, self.size)

        # Pre-calculate Distance Graphs (Plant <-> Plant)
        self.dist_plant_plant = {p: {} for p in self.plant_positions}
        for i, p1 in enumerate(self.plant_positions):
            for p2 in self.plant_positions[i+1:]:
                d = self.plant_distances[p1].get(p2, float('inf'))
                self.dist_plant_plant[p1][p2] = d
                self.dist_plant_plant[p2][p1] = d

        robot_states = self._build_robot_states(initial["Robots"])
        plant_states = self._build_plant_states(initial["Plants"])
        tap_states = self._build_tap_states(initial["Taps"])
        total_remaining = sum(initial["Plants"].values())

        initial_state = (robot_states, plant_states, tap_states, total_remaining)
        search.Problem.__init__(self, initial_state)

    @staticmethod
    def _bfs_map(start_pos, walls, size):
        queue = [(start_pos, 0)]
        distances = {start_pos: 0}
        visited = {start_pos}
        idx = 0
        while idx < len(queue):
            (curr_r, curr_c), dist = queue[idx]
            idx += 1
            for move in _MOVES.values():
                nr, nc = curr_r + move[0], curr_c + move[1]
                if 0 <= nr < size[0] and 0 <= nc < size[1]:
                    if (nr, nc) not in walls and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        distances[(nr, nc)] = dist + 1
                        queue.append(((nr, nc), dist + 1))
        return distances

    def _build_tunnel_map(self):
        tunnel_map = set()
        rows, cols = self.size
        for r in range(rows):
            for c in range(cols):
                if (r, c) in self.walls: continue
                degree = 0
                for move in _MOVES.values():
                    nr, nc = r + move[0], c + move[1]
                    if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in self.walls:
                        degree += 1
                if degree <= 2:
                    tunnel_map.add((r, c))
        return frozenset(tunnel_map)

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {rid: cap for rid, (_, _, _, cap) in robots_data.items()}

    def _build_robot_states(self, robots_data):
        return frozenset((rid, r, c, load) for rid, (r, c, load, _) in sorted(robots_data.items()))

    def _build_plant_states(self, plants_data):
        return tuple(0 for _ in self.plant_positions)

    def _build_tap_states(self, taps_data):
        return tuple(taps_data.get(pos, 0) for pos in self.tap_positions)

    def successor(self, state):
        robot_states, plant_states, tap_states, total_remaining = state
        successors = []
        occupied = {(r, c) for (_, r, c, _) in robot_states}
        num_robots = len(robot_states)
        
        safe_to_prune = True
        if num_robots > 1:
            robot_positions = list(occupied)
            min_dist = float('inf')
            for i in range(len(robot_positions)):
                for j in range(i + 1, len(robot_positions)):
                    d = abs(robot_positions[i][0] - robot_positions[j][0]) + \
                        abs(robot_positions[i][1] - robot_positions[j][1])
                    if d < min_dist: min_dist = d
            if min_dist <= 6: safe_to_prune = False

        for robot_entry in robot_states:
            robot_id, r, c, load = robot_entry
            
            is_blocked = False
            if num_robots > 1:
                for move in _MOVES.values():
                    if (r + move[0], c + move[1]) in occupied:
                        is_blocked = True; break
            
            in_tunnel = (r, c) in self.tunnel_map
            use_strict_pruning = safe_to_prune and not is_blocked and in_tunnel and (num_robots == 1)

            if use_strict_pruning:
                pour_succ = self._get_pour_successor(state, robot_entry)
                if pour_succ:
                    successors.extend(pour_succ)
                    continue
                load_succ = self._get_load_successor(state, robot_entry)
                if load_succ:
                    successors.extend(load_succ)
                    continue
                strict_moves = self._get_strict_movement_successors(state, robot_entry)
                if strict_moves:
                    successors.extend(strict_moves)
                else:
                    successors.extend(self._get_movement_successors(state, robot_entry))
            else:
                successors.extend(self._get_pour_successor(state, robot_entry))
                successors.extend(self._get_load_successor(state, robot_entry))
                successors.extend(self._get_movement_successors(state, robot_entry))

        return successors

    def _get_strict_movement_successors(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        successors = []
        occupied = {(rid, rr, cc) for (rid, rr, cc, _) in robot_states if rid != robot_id}

        active_maps = []
        capacity = self.robots_capacities.get(robot_id, 0)
        
        if load < capacity and load < total_remaining:
             for i, pos in enumerate(self.tap_positions):
                if tap_states[i] > 0: active_maps.append(self.tap_distances[pos])

        if load > 0:
            for i, pos in enumerate(self.plant_positions):
                target = self.plants_targets.get(pos, 0)
                if plant_states[i] < target: active_maps.append(self.plant_distances[pos])

        if not active_maps: return [] 

        valid_objectives = []
        for d_map in active_maps:
            d = d_map.get((r, c), float('inf'))
            if d != float('inf'): valid_objectives.append((d_map, d))

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            
            is_occ = False
            for _, orr, occ in occupied:
                if nr == orr and nc == occ: is_occ = True; break
            if is_occ: continue

            is_productive = False
            for d_map, cur_dist in valid_objectives:
                if d_map.get((nr, nc), float('inf')) < cur_dist:
                    is_productive = True
                    break
            
            if is_productive:
                new_robot_entry = (robot_id, nr, nc, load)
                new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
                new_state = (new_robot_states, plant_states, tap_states, total_remaining)
                successors.append(((robot_id, action_name), new_state))
                
        return successors

    def _get_movement_successors(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        successors = []
        occupied = {(rid, rr, cc) for (rid, rr, cc, _) in robot_states if rid != robot_id}

        for action_name, (dr, dc) in _MOVES.items():
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.size[0] and 0 <= nc < self.size[1]): continue
            if (nr, nc) in self.walls: continue
            
            is_occ = False
            for _, orr, occ in occupied:
                if nr == orr and nc == occ: is_occ = True; break
            if is_occ: continue

            new_robot_entry = (robot_id, nr, nc, load)
            new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
            new_state = (new_robot_states, plant_states, tap_states, total_remaining)
            successors.append(((robot_id, action_name), new_state))
        return successors

    def _get_load_successor(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        if (r, c) not in self.tap_positions: return []
        if load >= total_remaining: return [] 
        tidx = self.tap_positions.index((r, c))
        remaining_tap = tap_states[tidx]
        if remaining_tap <= 0: return []
        capacity = self.robots_capacities.get(robot_id, 0)
        if load >= capacity: return []
        new_robot_entry = (robot_id, r, c, load + 1)
        new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
        new_tap_states = tuple((remaining_tap - 1) if i == tidx else amt for i, amt in enumerate(tap_states))
        return [((robot_id, "LOAD"), (new_robot_states, plant_states, new_tap_states, total_remaining))]

    def _get_pour_successor(self, state, robot_entry):
        robot_states, plant_states, tap_states, total_remaining = state
        robot_id, r, c, load = robot_entry
        if load <= 0: return []
        if (r, c) not in self.plant_positions: return []
        pidx = self.plant_positions.index((r, c))
        poured = plant_states[pidx]
        target = self.plants_targets.get((r, c), 0)
        if poured >= target: return []
        new_robot_entry = (robot_id, r, c, load - 1)
        new_robot_states = (robot_states - {robot_entry}) | {new_robot_entry}
        new_plant_states = tuple((poured + 1) if i == pidx else amt for i, amt in enumerate(plant_states))
        new_total_remaining = total_remaining - 1
        return [((robot_id, "POUR"), (new_robot_states, new_plant_states, tap_states, new_total_remaining))]

    def goal_test(self, state):
        return state[3] == 0

    def _calculate_mst_simple(self, state):
        """
        Calculates MST Lower Bound on Movement.
        Nodes: Robots and Unwatered Plants.
        """
        robot_states, plant_states, _, _ = state
        
        # 1. Identify Target Nodes (Unsatisfied Plants)
        mst_targets = []
        for i, pos in enumerate(self.plant_positions):
            # Only add plants that still NEED water
            if plant_states[i] < self.plants_targets.get(pos, 0):
                mst_targets.append(pos)
        
        if not mst_targets:
            return 0

        # 2. Prim's Algorithm Initialization
        min_dists = {}
        for target in mst_targets:
            # Find closest robot to this plant
            best_d = float('inf')
            for (_, rr, rc, _) in robot_states:
                d = self.plant_distances[target].get((rr, rc), float('inf'))
                if d < best_d:
                    best_d = d
            min_dists[target] = best_d

        # 3. Prim's Main Loop
        mst_cost = 0
        unvisited = set(mst_targets)
        
        while unvisited:
            # Extract plant with min distance to the current tree
            u = min(unvisited, key=lambda k: min_dists[k])
            dist_u = min_dists[u]
            
            if dist_u == float('inf'):
                break 
                
            mst_cost += dist_u
            unvisited.remove(u)
            
            # Update neighbors
            for v in unvisited:
                d = self.dist_plant_plant[u][v]
                if d < min_dists[v]:
                    min_dists[v] = d
                    
        return mst_cost

    def h_astar(self, node):
        """
        Strictly Admissible Heuristic:
        Cost = MST (Movement) + Pours (Guaranteed)
        
        NOTE: We do NOT add cost for LOAD actions here.
        While logically we must load, adding explicit cost + MST cost
        can result in inadmissibility in edge cases where the path 
        overlaps. Removing it is safer.
        """
        state = node.state
        robot_states, _, _, total_remaining = state
        
        # 1. MST Cost (Strict Lower Bound on Movement)
        mst_cost = self._calculate_mst_simple(state)
        
        # 2. Interaction Cost (Pours only)
        interaction_cost = total_remaining

        return mst_cost + interaction_cost

    def h_gbfs(self, node):
        state = node.state
        robot_states, _, _, total_remaining = state
        if total_remaining == 0: return 0
        
        carried = sum(load for (_, _, _, load) in robot_states)
        water_needed = total_remaining + max(0, total_remaining - carried)
        
        mst_cost = self._calculate_mst_simple(state)
        
        return water_needed + mst_cost

def create_watering_problem(game):
    return WateringProblem(game)