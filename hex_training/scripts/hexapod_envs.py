from environments.hexapod_env_idle import HexapodIdle
import gymnasium as gym
#register the training environment in the gym as an available one
reg = gym.register(
    id='Hexapod-v0-idle',
    entry_point='hexapod_envs:HexapodIdle',
    max_episode_steps=50
    )

