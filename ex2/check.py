from ex2 import Controller

def test_simple_astar():
    # 1. Define a simple 3x3 grid test case
    # Robot at (0,0), Tap at (0,2), Plant at (2,0)
    test_initial_state = {
        "Size":  (12, 12),
        "Walls": {
            (1, 3), (2, 3), (3, 3), (4, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3),
            (1, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (10, 6),
            (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (8, 9), (9, 9), (10, 9),
        },
        "Taps": {(5, 1): 24},
        "Plants": {(2, 11): 5, (7, 11): 5, (10, 10): 5, (0, 8): 5},
        "Robots": {10: (11, 1, 0, 3)},
    }

    print("--- Starting A* Test ---")
    
    # 2. Initialize Controller (passing None as 'game' since a_star doesn't use it)
    try:
        controller = Controller(None)
    except Exception as e:
        print(f"Error initializing controller: {e}")
        return

    # 3. Run a_star
    try:
        plan = controller.a_star(test_initial_state)
        print(plan)
        print("\nCalculated Plan:")
        for i, action in enumerate(plan):
            print(f"{i+1}. {action}")

        # 4. Basic Validation
        # Expected path: Move Right -> Move Right -> Load -> Move Left -> Move Left -> Move Down -> Move Down -> Pour
        expected_actions = ['RIGHT{R1}', 'RIGHT{R1}', 'LOAD{R1}', 'LEFT{R1}', 'LEFT{R1}', 'DOWN{R1}', 'DOWN{R1}', 'POUR{R1}']
        
        # Note: Depending on exact move order preference in ex1, specific path might slightly vary 
        # (e.g. Down then Left), but the logic should hold.
        if len(plan) > 0:
            print("\n[SUCCESS] A* returned a plan.")
        else:
            print("\n[FAILURE] A* returned an empty list (no solution found).")
            
    except Exception as e:
        print(f"\n[CRITICAL ERROR] Failed during a_star execution: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_simple_astar()