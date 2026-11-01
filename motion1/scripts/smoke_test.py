import _paths  # noqa: F401  (puts src/ on the path)
import numpy as np
from gymnasium.utils.env_checker import check_env

from humanoid_grasp_rl import JaxDeliveryEnv, JaxGraspEnv, JaxStackEnv
from humanoid_grasp_rl.demonstration import scripted_palm_place


for env_class in (JaxGraspEnv, JaxDeliveryEnv):
    env = env_class()
    check_env(env, skip_render_check=True)
    observation, info = env.reset(seed=7)
    start = env.grasp_centre.copy()
    # One step of +z on the position channel moves the commanded pose 10 mm up.
    action = np.zeros(4, dtype=np.float32)
    action[2] = 1.0
    env.step(action)
    np.testing.assert_allclose(env._pose_target_position[2], start[2] + 0.01, atol=1e-9)
    for _ in range(100):
        observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
        assert env.observation_space.contains(observation)
        if terminated or truncated:
            observation, info = env.reset()
    env.close()

# The scripted controller delivers a cube, and builds a two-cube tower, from the fixed layouts.
for env, steps in ((JaxDeliveryEnv(), 700), (JaxStackEnv(cubes=2), 1400)):
    env.reset(seed=0)
    landed_z, rise = None, 0.0
    for step in range(steps):
        _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
        # The cube must stay down while the hand lets go. It once rose 4-9 mm: the hand was pulled back up
        # with the pads still closed and took the cube with it.
        if env._palm_phase in ("settle", "let go"):
            z = float(env.data.xpos[env.object_body_ids[env.moving]][2])
            landed_z = z if landed_z is None else landed_z
            rise = max(rise, z - landed_z)
        else:
            landed_z = None
        if terminated or truncated:
            break
    assert info["is_success"] and info["success_now"], f"{type(env).__name__}: not completed"
    assert rise < 0.002, f"{type(env).__name__}: the placed cube rose {rise * 1000:.1f} mm while the hand let go"
    env.close()
print("smoke test passed")
