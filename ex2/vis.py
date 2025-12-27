import ast

def visualize_state(input_str):
    # --- 1. Define the Problem Rules (Rewards) ---
    # These values are taken from 'problem_new2_version1' which matches your state.
    # In a real run, you would pass the 'problem' dictionary to this function.
    problem_config = {
        "goal_reward": 18,
        "plants_reward": {
            (0, 0): [5, 7],  # Possible rewards for plant at (0,0)
            (4, 5): [5, 7],  # Possible rewards for plant at (4,5)
        }
    }

    # --- 2. Parse the State ---
    clean_str = input_str.strip().replace("State: ", "")
    try:
        state = ast.literal_eval(clean_str)
    except:
        print("Error parsing state string.")
        return

    robots, plants, taps, total_need = state

    # --- 3. Calculate Maximal Reward ---
    # Max Reward = Goal + Sum(Max_Reward_Per_Drop * Drops_Needed)
    current_potential = problem_config["goal_reward"]
    
    print("\n--- Reward Calculation (Best Case) ---")
    print(f"Goal Reward: {problem_config['goal_reward']}")
    
    for pos, need in plants:
        if pos in problem_config["plants_reward"]:
            # We assume best case (max) for "Maximal Reward"
            possible_rewards = problem_config["plants_reward"][pos]
            best_drop = max(possible_rewards)
            plant_total = best_drop * need
            current_potential += plant_total
            print(f"Plant {pos}: Need {need} x Max Reward {best_drop} = {plant_total}")
        else:
            print(f"Plant {pos}: Unknown reward (config missing)")

    # --- 4. Visualization Logic ---
    all_coords = [r[1] for r in robots] + [p[0] for p in plants] + [t[0] for t in taps]
    if not all_coords: max_r, max_c = 5, 5
    else:
        max_r = max(c[0] for c in all_coords) + 1
        max_c = max(c[1] for c in all_coords) + 1
    
    grid = [[" . " for _ in range(max_c)] for _ in range(max_r)]

    C_PLANT = "\033[91m" # Red
    C_TAP   = "\033[94m" # Blue
    C_ROBOT = "\033[92m" # Green
    C_BOLD  = "\033[1m"
    RESET   = "\033[0m"

    for pos, water in taps:
        grid[pos[0]][pos[1]] = f"{C_TAP}T{water:<2}{RESET}"
    for pos, need in plants:
        grid[pos[0]][pos[1]] = f"{C_PLANT}P{need:<2}{RESET}"
    for rid, pos, load in robots:
        grid[pos[0]][pos[1]] = f"{C_ROBOT}R{rid:<2}{RESET}"

    print(f"\n{C_BOLD}Visual State Representation:{RESET}")
    print(f"Grid Size: {max_r}x{max_c}")
    print("-" * (max_c * 5 + 1))
    for row in grid:
        print("| " + " ".join(row) + "|")
    print("-" * (max_c * 5 + 1))
    print(f"Legend: {C_ROBOT}R=Robot{RESET}, {C_PLANT}P=Plant{RESET}, {C_TAP}T=Tap{RESET}")
    
    # Print the calculated max reward at the bottom
    print(f"\n{C_BOLD}>>> MAXIMAL THEORETICAL REWARD: {current_potential} <<<{RESET}")

# --- Run ---
raw_input = "State: (((10, (2, 1), 0), (11, (1, 1), 1)), (((0, 1), 1), ((4, 5), 6)), (((2, 2), 10),), 7)"
visualize_state(raw_input)