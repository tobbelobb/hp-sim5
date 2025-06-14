import pytest
import numpy as np

from python.ecs import World, PositionComponent, BallTagComponent
from flipper_server_warp import setup_scene, ScoreComponent


def get_game_state_for_test(world):
    """Helper function to extract ball positions and score from the world."""
    state = {'balls': [], 'score': 0}

    ball_entities = world.query([BallTagComponent, PositionComponent])
    for ball_id in ball_entities:
        pos_comp = world.get_component(ball_id, PositionComponent)
        state['balls'].append({'id': ball_id, 'y': pos_comp.pos[1]})

    score_query = world.query([ScoreComponent])
    if score_query:
        score_comp = world.get_component(score_query[0], ScoreComponent)
        state['score'] = score_comp.value

    return state


def test_flipper_warp_autonomous_run_and_settle():
    world = World()
    setup_scene(world)

    pause_state = world.get_resource('pauseState')
    pause_state.paused = False

    EXPECTED_SCORE = 11  # Expect same score as non-warp solver
    MAX_SIMULATION_STEPS = 100 * 300
    POLLING_INTERVAL_STEPS = 1
    FLIPPER_Y_LINE = 0.05
    SCORE_GUARDRAIL = 100

    dt = world.get_resource('dt')

    test_passed = False
    settled = False

    for step in range(MAX_SIMULATION_STEPS):
        world.update(dt)
        if step > 0 and step % POLLING_INTERVAL_STEPS == 0:
            game_state = get_game_state_for_test(world)
            balls = game_state['balls']
            score = game_state['score']

            if score > SCORE_GUARDRAIL:
                pytest.fail(f"Test failed: Score exceeded {SCORE_GUARDRAIL}. Current score: {score}")

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
                if score == EXPECTED_SCORE:
                    test_passed = True
                else:
                    pytest.fail(f"Test failed: Balls settled below flippers, but score is {score} (expected {EXPECTED_SCORE}).")
                break

    if not settled:
        final_game_state = get_game_state_for_test(world)
        pytest.fail(
            f"Test failed: Timeout. Balls did not settle below flippers within {int(MAX_SIMULATION_STEPS / POLLING_INTERVAL_STEPS)}s. Final score: {final_game_state['score']}"
        )

    assert settled, "Balls should have settled"
    assert test_passed, "Test condition for score was not met."

    final_score = get_game_state_for_test(world)['score']
    assert final_score == EXPECTED_SCORE, f"Final score should be {EXPECTED_SCORE}"
