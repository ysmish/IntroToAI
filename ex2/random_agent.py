import random

class Controller:
    def __init__(self, problem):
        """
        Initialize the random controller.
        """
        # Extract the model dictionary (handling the API variations)
        if hasattr(problem, 'get_problem'):
            self.model = problem.get_problem()
        elif hasattr(problem, 'get_model'):
            self.model = problem.get_model()
        elif hasattr(problem, '_model'):
            self.model = problem._model
        else:
            self.model = problem # Fallback if it is already the dict
            
        # Parse static data needed for legality checks
        self.walls = set(self.model.get('Walls', []))
        self.rows, self.cols = self.model['Size']
        
        # Robot capacities map ID -> Capacity
        self.robot_capacities = {
            rid: val[3] for rid, val in self.model['Robots'].items()
        }

    def get_legal_actions(self, state):
        """
        Generates a list of all legal actions for all robots in the current state.
        (Identical logic to ex2.py to ensure the random agent doesn't crash)
        """
        robots, plants, taps, _ = state
        legal_actions = []

        # Convert tuples to efficient lookups
        plant_locs = {p[0]: p[1] for p in plants} # (r,c) -> need
        tap_locs = {t[0]: t[1] for t in taps}     # (r,c) -> water
        
        # Current robot positions (to avoid collisions)
        robot_positions = {r[1] for r in robots}

        for robot in robots:
            rid, (r, c), load = robot
            capacity = self.robot_capacities[rid]

            # --- 1. MOVE Actions ---
            moves = [('UP', (-1, 0)), ('DOWN', (1, 0)), ('LEFT', (0, -1)), ('RIGHT', (0, 1))]
            for move_name, (dr, dc) in moves:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if (nr, nc) not in self.walls:
                        if (nr, nc) not in robot_positions:
                            legal_actions.append(f"{move_name} ({rid})")

            # --- 2. LOAD Action ---
            if (r, c) in tap_locs and tap_locs[(r, c)] > 0 and load < capacity:
                legal_actions.append(f"LOAD ({rid})")

            # --- 3. POUR Action ---
            if (r, c) in plant_locs and plant_locs[(r, c)] > 0 and load > 0:
                legal_actions.append(f"POUR ({rid})")

        # --- 4. RESET Action ---
        # Included in random pool. 
        # Note: A pure random agent might reset often, making it easy to beat.
        legal_actions.append("RESET")
        
        return legal_actions

    def choose_next_action(self, state):
        """
        Selects a legal action uniformly at random.
        """
        legal_actions = self.get_legal_actions(state)
        
        if not legal_actions:
            return "RESET"

        return random.choice(legal_actions)