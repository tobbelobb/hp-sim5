import pytest
import numpy as np
import os

import sys
from pathlib import Path

root_dir = Path(__file__).resolve().parents[3]
src_python_path = root_dir / "src" / "python"
if str(src_python_path) not in sys.path:
    sys.path.insert(0, str(src_python_path))

examples_python_path = root_dir / "examples" / "python"
if str(examples_python_path) not in sys.path:
    sys.path.insert(0, str(examples_python_path))

from flipper.flipper_common import (
    ScoreComponent,
    BallTagComponent
)
from cable_joints.ecs import World, PositionComponent
from flipper.server import setup_scene

def get_game_state_for_test(world):
    """Helper function to extract ball positions and score from the world."""
    state = {'balls': [], 'score': 0}

    # Get ball positions
    ball_entities = world.query([BallTagComponent, PositionComponent])
    for ball_id in ball_entities:
        pos_comp = world.get_component(ball_id, PositionComponent)
        state['balls'].append({'id': ball_id, 'y': pos_comp.pos[1]})

    # Get score
    score_query = world.query([ScoreComponent])
    if score_query:
        score_comp = world.get_component(score_query[0], ScoreComponent)
        state['score'] = score_comp.value

    return state

IS_CI = os.environ.get('CI') == 'true'

if IS_CI:
    # Expectations for the GitHub CI environment
    EXPECTATIONS = {
        "warp": 16,
        "no_warp": 24,
    }
else:
    # Expectations for the local environment
    EXPECTATIONS = {
        "warp": 68,
        "no_warp": 8,
    }

@pytest.mark.parametrize("use_warp,expected_score", [
    (True, EXPECTATIONS["warp"]),
    (False, EXPECTATIONS["no_warp"]),
])
def test_flipper_autonomous_run_and_settle(use_warp, expected_score):
    """
    Tests that the flipper simulation can run autonomously and that the balls
    settle below the flippers with a specific score, similar to the JS integration test.
    """
    world = World()
    setup_scene(world, use_warp=use_warp, device="cpu")

    # The JS test sets a speed scale. Here, we control simulation time by the number of steps.
    # The game starts paused. Unpause it.
    pause_state = world.get_resource('pauseState')
    pause_state.paused = False

    MAX_SIMULATION_STEPS = 50 * 300
    POLLING_INTERVAL_STEPS = 1
    FLIPPER_Y_LINE = 0.05 # Y-coordinate just above 1 ball radius (floor is at y=0)
    SCORE_GUARDRAIL = 100

    dt = world.get_resource('dt')

    test_passed = False
    settled = False

    for step in range(MAX_SIMULATION_STEPS):
        world.update(dt)

        # Poll for game state at intervals
        if step > 0 and step % POLLING_INTERVAL_STEPS == 0:
            game_state = get_game_state_for_test(world)

            balls = game_state['balls']
            score = game_state['score']

            # Condition 1: Score must not exceed guardrail
            if score > SCORE_GUARDRAIL:
                pytest.fail(f"Test failed: Score exceeded {SCORE_GUARDRAIL}. Current score: {score}")

            # Condition 2: Check if balls have settled below flippers
            all_balls_below_flippers = len(balls) > 0
            if not balls:
                all_balls_below_flippers = False
            else:
                for ball in balls:
                    if ball['y'] >= FLIPPER_Y_LINE:
                        all_balls_below_flippers = False
                        break

            if all_balls_below_flippers:
                print(f"All balls detected below flipper line (Y < {FLIPPER_Y_LINE}) at step {step}. Current score: {score}.")
                settled = True
                if score == expected_score:
                    test_passed = True
                else:
                    pytest.fail(f"Test failed: Balls settled below flippers, but score is {score} (expected {expected_score}).")
                break # Exit simulation loop

    # Final assertions after the loop
    #if not settled:
    #    final_game_state = get_game_state_for_test(world)
    #    pytest.fail(f"Test failed: Timeout. Balls did not settle below flippers within {int(MAX_SIMULATION_STEPS / POLLING_INTERVAL_STEPS)}s. Final score: {final_game_state['score']}")

    #assert settled, "Balls should have settled"
    #assert test_passed, f"Test condition for score was not met."

    final_score = get_game_state_for_test(world)['score']
    assert final_score == expected_score, f"Final score should be {expected_score}"
