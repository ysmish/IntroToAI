import ext_plant
import ex2
import numpy as np
import time

# ANSI color codes for nicer terminal output
RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[96m"
MAGENTA = "\033[95m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
SEP = "\n" + ("=" * 60)


def solve(game: ext_plant.Game, run_idx: int):
    policy = ex2.Controller(game)
    for i in range(game.get_max_steps()):
        game.submit_next_action(
            chosen_action=policy.choose_next_action(game.get_current_state())
        )
        if game.get_done():
            break

    r = game.get_current_reward()
    state = game.get_current_state()
    state_str = str(state)

    finished_txt = "SUCCESS" if state[-1] else "FAILED"
    color = GREEN if state[-1] else RED

    print(
        f"Run {run_idx:2d}: {BOLD}{YELLOW}Reward: {r:3d}{RESET} | Steps: {game.get_max_steps():2d} | {color}{finished_txt}{RESET} \nState: {state_str}"
    )
    return r


problem_pdf = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.9,
    },
    "goal_reward": 10,
    "plants_reward": {
        (0, 2): [1, 2, 3, 4],
        (2, 0): [1, 2, 3, 4],
    },
    "seed": 45,
    "horizon": 30,
}

problem_pdf2 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {
        10: 0.9,
        11: 0.8,
    },
    "goal_reward": 12,
    "plants_reward": {
        (0, 2): [1, 3, 5, 7],
        (2, 0): [1, 2, 3, 4],
    },
    "seed": 45,
    "horizon": 35,
}

problem_pdf3 = {
    "Size": (3, 3),
    "Walls": {(0, 1), (2, 1)},
    "Taps": {(1, 1): 6},
    "Plants": {(2, 0): 2, (0, 2): 3},
    "Robots": {10: (1, 0, 0, 2), 11: (1, 2, 0, 2)},
    "robot_chosen_action_prob": {
        10: 0.7,
        11: 0.6,
    },
    "goal_reward": 30,
    "plants_reward": {
        (0, 2): [1, 2, 3, 4],
        (2, 0): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 30,
}

problem_new1_version1 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 3,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.9,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 30,
}

problem_new1_version2 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 3,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.6,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 70,
}

problem_new1_version3 = {
    "Size": (5, 6),
    "Walls": {
        # block some middle cells to create a kind of corridor
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
    },
    "Taps": {
        (2, 2): 12,
    },
    "Plants": {
        (0, 1): 2,
        (4, 5): 6,
    },
    "Robots": {
        10: (2, 1, 0, 6),
        11: (2, 4, 0, 3),
    },
    "robot_chosen_action_prob": {
        10: 0.6,
        11: 0.95,
    },
    "goal_reward": 30,
    "plants_reward": {
        (4, 5): [1, 2, 3, 4],
        (0, 1): [10, 11, 12, 13],
    },
    "seed": 45,
    "horizon": 30,
}

problem_new2_version1 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 30,
}


problem_new2_version2 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 70,
}

problem_new2_version3 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 20,
    "plants_reward": {
        (0, 0): [5, 7, 9],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 30,
}

problem_new2_version4 = {
    "Size": (5, 6),
    "Walls": {
        # corridor shifted up
        (0, 2),
        (0, 3),
        (2, 2),
        (2, 3),
    },
    "Taps": {
        (1, 2): 10,  # upper tap
        (3, 3): 10,  # lower tap
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (4, 5): 5,  # bottom-right
    },
    "Robots": {
        10: (1, 1, 0, 5),  # near upper tap, cap 3
        11: (3, 4, 0, 4),  # near lower tap, cap 2
    },
    "robot_chosen_action_prob": {
        10: 0.7,
        11: 0.95,
    },
    "goal_reward": 18,
    "plants_reward": {
        (0, 0): [5, 7],
        (4, 5): [5, 7],
    },
    "seed": 45,
    "horizon": 40,
}

problem_new3_version1 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 10,  # upper-right corrido
        (9, 0): 10,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.95,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 30,
}

problem_new3_version2 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 10,  # upper-right corrido
        (9, 0): 10,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.8,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 50,
}

problem_new3_version3 = {
    "Size": (10, 4),
    "Walls": {
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),
        (4, 1),
        (6, 1),
        (7, 1),
        (8, 1),
        (9, 1),
        (4, 2),
        (4, 3),
        (6, 2),
        (6, 3),
    },
    # Tap on the left side, with enough water
    "Taps": {
        (5, 3): 20,
    },
    # Plants on the far right, all need water
    "Plants": {
        (0, 0): 5,  # upper-right corrido
        (9, 0): 5,
    },
    # Single robot, small capacity → many long trips through the maze
    "Robots": {
        10: (2, 0, 0, 2),  # bottom-left area near the tap side
        11: (7, 0, 0, 20),  # bottom-left area near the tap side
    },
    "robot_chosen_action_prob": {
        10: 0.95,
        11: 0.0001,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (9, 0): [1, 3],
    },
    "seed": 45,
    "horizon": 70,
}
# reset ?
problem_new4_version1 = {
    "Size": (10, 10),
    "Walls": set(),  # completely open grid
    "Taps": {
        (8, 8): 24,
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (0, 9): 5,  # top-right
        (9, 0): 5,  # bottom-left
        (9, 9): 5,  # bottom-right
        # total need = 20
    },
    "Robots": {
        10: (8, 9, 0, 5),
    },
    "robot_chosen_action_prob": {
        10: 0.95,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (0, 9): [1, 3],
        (9, 0): [1, 3],
        (9, 9): [1, 3],
    },
    "seed": 45,
    "horizon": 70,
}

# reset ?
problem_new4_version2 = {
    "Size": (10, 10),
    "Walls": set(),  # completely open grid
    "Taps": {
        (8, 8): 24,
    },
    "Plants": {
        (0, 0): 5,  # top-left
        (0, 9): 5,  # top-right
        (9, 0): 5,  # bottom-left
        (9, 9): 5,  # bottom-right
        # total need = 20
    },
    "Robots": {
        10: (8, 9, 0, 5),
    },
    "robot_chosen_action_prob": {
        10: 0.85,
    },
    "goal_reward": 9,
    "plants_reward": {
        (0, 0): [1, 3],
        (0, 9): [1, 3],
        (9, 0): [1, 3],
        (9, 9): [1, 3],
    },
    "seed": 45,
    "horizon": 40,
}


def main():
    debug_mode = False
    n_runs = 30
    # Fix horizon
    total_reward = 0.0
    problems = [
        problem_pdf,
        problem_pdf2,
        problem_pdf3,
        problem_new1_version1,
        problem_new1_version2,
        problem_new1_version3,
        problem_new2_version1,
        problem_new2_version2,
        problem_new2_version3,
        problem_new2_version4,
        problem_new3_version1,
        problem_new3_version2,
        problem_new3_version3,
        problem_new4_version1,
        problem_new4_version2,
    ]

    # Map readable names to problems for clearer summary output
    problem_names = [
        "problem_pdf",
        "problem_pdf2",
        "problem_pdf3",
        "problem_new1_version1",
        "problem_new1_version2",
        "problem_new1_version3",
        "problem_new2_version1",
        "problem_new2_version2",
        "problem_new2_version3",
        "problem_new2_version4",
        "problem_new3_version1",
        "problem_new3_version2",
        "problem_new3_version3",
        "problem_new4_version1",
        "problem_new4_version2",
    ]

    # Zip names with problems and slice the same range used above
    named_problems = list(zip(problem_names, problems))[0:]

    # Baseline averages (reward_avg, time_avg) provided by user for comparison
    baseline_map = {
        "problem_pdf": (21.766667, 1.296914),
        "problem_pdf2": (33.566667, 1.173343),
        "problem_pdf3": (40.366667, 1.208422),
        "problem_new1_version1": (62.966667, 7.863858),
        "problem_new1_version2": (87.500000, 16.394542),
        "problem_new1_version3": (26.833333, 6.774192),
        "problem_new2_version1": (26.600000, 12.010865),
        "problem_new2_version2": (86.066667, 27.054848),
        "problem_new2_version3": (46.533333, 10.588311),
        "problem_new2_version4": (39.733333, 15.901740),
        "problem_new3_version1": (2.933333, 0.898599),
        "problem_new3_version2": (4.033333, 1.386519),
        "problem_new3_version3": (5.900000, 2.025747),
        "problem_new4_version1": (38.033333, 2.906778),
        "problem_new4_version2": (16.566667, 1.627962),
    }

    # Collect per-problem summaries
    summaries = []
    for idx, (pname, problem) in enumerate(named_problems, start=1):
        print(
            f"\n{SEP}\n{BOLD}{MAGENTA}--- Running {pname} (problem index slice item {idx}) ---{RESET}"
        )
        total_reward = 0.0
        start_time = time.time()
        # Use provided baseline for comparison instead of computing an "ideal"
        baseline_reward, baseline_time = baseline_map.get(pname, (None, None))
        horizon = problem.get("horizon", 0)
        for seed in range(n_runs):
            # Set a different random seed each run
            problem["seed"] = seed

            # Create a fresh game for this run
            game = ext_plant.create_pressure_plate_game((problem, debug_mode))

            # Solve and accumulate reward
            run_reward = solve(game, seed)
            total_reward += run_reward

        end_time = time.time()
        duration = end_time - start_time

        # Time limit check: 20 + 0.5 * horizon
        time_limit = 20 + 0.5 * horizon
        time_status = (
            f"{GREEN}PASS{RESET}"
            if duration <= time_limit
            else f"{RED}TIMEOUT (Limit: {time_limit}s){RESET}"
        )

        avg_reward = total_reward / n_runs
        summaries.append(
            (pname, avg_reward, duration, baseline_reward, baseline_time, time_status)
        )
        if baseline_reward:
            pct = (avg_reward / baseline_reward * 100) if baseline_reward > 0 else 0
            comp = (
                f"{GREEN}BETTER{RESET}"
                if avg_reward > baseline_reward
                else f"{RED}WORSE{RESET}"
            )
            print(
                f"\nAverage reward over {n_runs} runs: {avg_reward:.6f} ({pct:.1f}% of baseline {baseline_reward:.6f}) | {comp} than baseline"
            )
            print(
                f"Baseline time: {baseline_time}s | Current time: {duration:.2f}s | Status: {time_status}"
            )
        else:
            print(f"\nAverage reward over {n_runs} runs: {avg_reward}")
            print(f"Time taken: {duration:.2f}s | Status: {time_status}")

    # Final summary across all problems run
    print(f"\n{SEP}\n{BOLD}{CYAN}=== Summary (per problem) ==={RESET}")
    for pname, avg, dur, baseline_avg, baseline_time, t_status in summaries:
        avg_col = f"{GREEN}{avg:.2f}{RESET}" if avg >= 0 else f"{RED}{avg:.2f}{RESET}"
        if baseline_avg:
            pct = (avg / baseline_avg * 100) if baseline_avg > 0 else 0
            comp = (
                f"{GREEN}BETTER{RESET}" if avg > baseline_avg else f"{RED}WORSE{RESET}"
            )
            baseline_str = f"baseline {YELLOW}{baseline_avg:.2f}{RESET} (time {YELLOW}{baseline_time:.2f}s{RESET})"
        else:
            pct = 0
            comp = ""
            baseline_str = "no baseline"
        print(
            f"{BOLD}{pname}{RESET}: average = {avg_col} ({pct:.1f}% of {baseline_str}) | time = {YELLOW}{dur:.2f}s{RESET} | {comp} {t_status}"
        )
    total_time = sum(d for (_, _, d, _, _, _) in summaries)
    print(f"{BOLD}Total time for all problems: {RESET}{YELLOW}{total_time:.2f}s{RESET}")
    # Overall average across all problems
    if summaries:
        overall_avg = sum(avg for (_, avg, _, _, _, _) in summaries) / len(summaries)
        print(
            f"{BOLD}Overall average across {len(summaries)} problems: {RESET}{YELLOW}{overall_avg:.2f}{RESET}"
        )


if __name__ == "__main__":
    main()
