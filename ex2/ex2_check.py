import ext_plant
import ex2
import numpy as np
import time

# ==========================================
#        HELPER FUNCTIONS
# ==========================================

def run_episode(game, policy_class, time_limit=None):
    """
    Runs a single episode with a specific policy class.
    """
    policy = policy_class(game) # Initialize the specific agent
    
    start_time = time.time()
    for _ in range(game.get_max_steps()):
        action = policy.choose_next_action(game.get_current_state())
        game.submit_next_action(chosen_action=action)
        if game.get_done():
            break
    end_time = time.time()
    
    duration = end_time - start_time
    violation = False
    if time_limit and duration > time_limit:
        violation = True
        
    return game.get_current_reward(), violation

def calculate_estimated_grade(score):
    """
    Maps the global average score to the grade based on the provided table image.
    """
    if score < 9.38: return 0
    if score <= 11.65: return "1 - 29"
    if score <= 13.45: return "30 - 44"
    if score <= 15.51: return "45 - 59"
    if score <= 17.9:  return 60         # Passing Threshold
    if score <= 19.81: return "61 - 65"
    if score <= 22.01: return "66 - 69"
    if score <= 25.39: return "70 - 75"
    if score <= 29.29: return "76 - 79"
    if score <= 33.79: return "80 - 85"
    if score <= 37.39: return "86 - 89"
    if score <= 39.84: return "90 - 93"
    if score <= 42.46: return "94 - 95"
    if score <= 45.24: return "96 - 97"
    if score <= 49.74: return 98
    if score <= 53.00: return 99
    return 100

# ==========================================
#            PROBLEM DEFINITIONS
# ==========================================

problem_pdf = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.9 },
    "goal_reward": 10,
    "plants_reward": { (0, 2) : [1,2,3,4], (2, 0) : [1,2,3,4] },
    "seed": 45,
    "horizon": 30,
}

problem_pdf2 = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob":{ 10: 0.9, 11: 0.8 },
    "goal_reward": 12,
    "plants_reward": { (0, 2) : [1,3,5,7], (2, 0) : [1,2,3,4] },
    "seed": 45,
    "horizon": 35,
}

problem_pdf3 = {
    "Size":   (3, 3),
    "Walls":  {(0, 1), (2, 1)},
    "Taps":   {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob":{ 10: 0.7, 11: 0.6 },
    "goal_reward": 30,
    "plants_reward": { (0, 2) : [1,2,3,4], (2, 0) : [10,11,12,13] },
    "seed": 45,
    "horizon": 30,
}

problem_new1_version1 = {
    "Size":  (5, 6),
    "Walls": { (1, 2), (1, 3), (3, 2), (3, 3) },
    "Taps": { (2, 2): 12 },
    "Plants": { (0, 1): 3, (4, 5): 6 },
    "Robots": { 10: (2, 1, 0, 6), 11: (2, 4, 0, 3) },
    "robot_chosen_action_prob":{ 10: 0.9, 11: 0.95 },
    "goal_reward": 30,
    "plants_reward": { (4, 5) : [1,2,3,4], (0, 1) : [10,11,12,13] },
    "seed": 45,
    "horizon": 30,
}

problem_new1_version2 = {
    "Size":  (5, 6),
    "Walls": { (1, 2), (1, 3), (3, 2), (3, 3) },
    "Taps": { (2, 2): 12 },
    "Plants": { (0, 1): 3, (4, 5): 6 },
    "Robots": { 10: (2, 1, 0, 6), 11: (2, 4, 0, 3) },
    "robot_chosen_action_prob":{ 10: 0.6, 11: 0.95 },
    "goal_reward": 30,
    "plants_reward": { (4, 5) : [1,2,3,4], (0, 1) : [10,11,12,13] },
    "seed": 45,
    "horizon": 70,
}

problem_new1_version3 = {
    "Size":  (5, 6),
    "Walls": { (1, 2), (1, 3), (3, 2), (3, 3) },
    "Taps": { (2, 2): 12 },
    "Plants": { (0, 1): 2, (4, 5): 6 },
    "Robots": { 10: (2, 1, 0, 6), 11: (2, 4, 0, 3) },
    "robot_chosen_action_prob":{ 10: 0.6, 11: 0.95 },
    "goal_reward": 30,
    "plants_reward": { (4, 5) : [1,2,3,4], (0, 1) : [10,11,12,13] },
    "seed": 45,
    "horizon": 30,
}

problem_new2_version1 = {
    "Size":  (5, 6),
    "Walls": { (0, 2), (0, 3), (2, 2), (2, 3) },
    "Taps": { (1, 2): 10, (3, 3): 10 },
    "Plants": { (0, 0): 5, (4, 5): 5 },
    "Robots": { 10: (1, 1, 0, 5), 11: (3, 4, 0, 4) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.95 },
    "goal_reward": 18,
    "plants_reward": { (0, 0) : [5,7], (4, 5) : [5,7] },
    "seed": 45,
    "horizon": 30,
}

problem_new2_version2 = {
    "Size":  (5, 6),
    "Walls": { (0, 2), (0, 3), (2, 2), (2, 3) },
    "Taps": { (1, 2): 10, (3, 3): 10 },
    "Plants": { (0, 0): 5, (4, 5): 5 },
    "Robots": { 10: (1, 1, 0, 5), 11: (3, 4, 0, 4) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.95 },
    "goal_reward": 18,
    "plants_reward": { (0, 0) : [5,7], (4, 5) : [5,7] },
    "seed": 45,
    "horizon": 70,
}

problem_new2_version3 = {
    "Size":  (5, 6),
    "Walls": { (0, 2), (0, 3), (2, 2), (2, 3) },
    "Taps": { (1, 2): 10, (3, 3): 10 },
    "Plants": { (0, 0): 5, (4, 5): 5 },
    "Robots": { 10: (1, 1, 0, 5), 11: (3, 4, 0, 4) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.95 },
    "goal_reward": 20,
    "plants_reward": { (0, 0) : [5,7,9], (4, 5) : [5,7] },
    "seed": 45,
    "horizon": 30,
}

problem_new2_version4 = {
    "Size":  (5, 6),
    "Walls": { (0, 2), (0, 3), (2, 2), (2, 3) },
    "Taps": { (1, 2): 10, (3, 3): 10 },
    "Plants": { (0, 0): 5, (4, 5): 5 },
    "Robots": { 10: (1, 1, 0, 5), 11: (3, 4, 0, 4) },
    "robot_chosen_action_prob":{ 10: 0.7, 11: 0.95 },
    "goal_reward": 18,
    "plants_reward": { (0, 0) : [5,7], (4, 5) : [5,7] },
    "seed": 45,
    "horizon": 40,
}

problem_new3_version1 = {
    "Size":  (10, 4),
    "Walls": { (0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3) },
    "Taps": { (5, 3): 20 },
    "Plants": { (0, 0): 10, (9, 0): 10 },
    "Robots": { 10: (2, 0, 0, 2), 11: (7, 0, 0, 20) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.95 },
    "goal_reward": 9,
    "plants_reward": { (0, 0) : [1,3], (9, 0) : [1,3] },
    "seed": 45,
    "horizon": 30,
}

problem_new3_version2 = {
    "Size":  (10, 4),
    "Walls": { (0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3) },
    "Taps": { (5, 3): 20 },
    "Plants": { (0, 0): 10, (9, 0): 10 },
    "Robots": { 10: (2, 0, 0, 2), 11: (7, 0, 0, 20) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.8 },
    "goal_reward": 9,
    "plants_reward": { (0, 0) : [1,3], (9, 0) : [1,3] },
    "seed": 45,
    "horizon": 50,
}

problem_new3_version3 = {
    "Size":  (10, 4),
    "Walls": { (0,1),(1, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1),(4,2), (4,3),(6,2), (6,3) },
    "Taps": { (5, 3): 20 },
    "Plants": { (0, 0): 5, (9, 0): 5 },
    "Robots": { 10: (2, 0, 0, 2), 11: (7, 0, 0, 20) },
    "robot_chosen_action_prob":{ 10: 0.95, 11: 0.0001 },
    "goal_reward": 9,
    "plants_reward": { (0, 0) : [1,3], (9, 0) : [1,3] },
    "seed": 45,
    "horizon": 70,
}

problem_new4_version1 = {
    "Size":  (10, 10),
    "Walls": set(),
    "Taps": { (8, 8): 24 },
    "Plants": { (0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5 },
    "Robots": { 10: (8, 9, 0, 5) },
    "robot_chosen_action_prob":{ 10: 0.95 },
    "goal_reward": 9,
    "plants_reward": { (0, 0) : [1,3], (0, 9) : [1,3], (9, 0) : [1,3], (9, 9) : [1,3] },
    "seed": 45,
    "horizon": 70,
}

problem_new4_version2 = {
    "Size":  (10, 10),
    "Walls": set(),
    "Taps": { (8, 8): 24 },
    "Plants": { (0, 0): 5, (0, 9): 5, (9, 0): 5, (9, 9): 5 },
    "Robots": { 10: (8, 9, 0, 5) },
    "robot_chosen_action_prob":{ 10: 0.85 },
    "goal_reward": 9,
    "plants_reward": { (0, 0) : [1,3], (0, 9) : [1,3], (9, 0) : [1,3], (9, 9) : [1,3] },
    "seed": 45,
    "horizon": 40,
}

# Register all problems in a dictionary
PROBLEM_REGISTRY = {
    "PDF_1": problem_pdf,
    "PDF_2": problem_pdf2,
    "PDF_3": problem_pdf3,
    "NEW1_v1": problem_new1_version1,
    "NEW1_v2": problem_new1_version2,
    "NEW1_v3": problem_new1_version3,
    "NEW2_v1": problem_new2_version1,
    "NEW2_v2": problem_new2_version2,
    "NEW2_v3": problem_new2_version3,
    "NEW2_v4": problem_new2_version4,
    "NEW3_v1": problem_new3_version1,
    "NEW3_v2": problem_new3_version2,
    "NEW3_v3": problem_new3_version3,
    "NEW4_v1": problem_new4_version1,
    "NEW4_v2": problem_new4_version2,
}

def main():
    debug_mode = False
    n_runs = 30 # Number of seeds per problem
    
    print(f"\n{'='*90}")
    print(f"{'ASSIGNMENT 2: FINAL GRADER (Table Based)':^90}")
    print(f"{'='*90}\n")
    
    problem_scores = []
    
    for name, problem in PROBLEM_REGISTRY.items():
        total_reward = 0.0
        time_violations = 0
        
        # Calculate time limit (20 + 0.5 * horizon)
        limit_seconds = 20 + (0.5 * problem["horizon"])
        
        print(f"Running {name:<10} ", end="", flush=True)

        for seed in range(n_runs):
            problem["seed"] = seed
            
            # Run Agent
            game = ext_plant.create_pressure_plate_game((problem, debug_mode))
            run_reward, v_time = run_episode(game, ex2.Controller, limit_seconds)
            
            total_reward += run_reward
            if v_time: time_violations += 1

            # Print dot every 5 seeds
            if seed % 5 == 0:
                print(".", end="", flush=True)

        avg_reward = total_reward / n_runs
        problem_scores.append(avg_reward)
        
        # Determine Warnings
        status_flag = "OK"
        if time_violations > 0: status_flag = f"SLOW ({time_violations})"
        
        print(f" Avg: {avg_reward:.2f} | {status_flag}")

    # --- CALCULATE FINAL GRADE ---
    # The grade is based on the average of all 15 problem averages
    final_average_score = sum(problem_scores) / len(problem_scores)
    estimated_grade = calculate_estimated_grade(final_average_score)

    print("\n" + "="*90)
    print(f"{'FINAL REPORT':^90}")
    print("-" * 90)
    print(f"{'Problem Count':<25} : {len(problem_scores)}")
    print(f"{'Global Average Reward':<25} : {final_average_score:.2f}")
    print(f"{'ESTIMATED GRADE':<25} : {estimated_grade}")
    print("="*90)

if __name__ == "__main__":
    main()