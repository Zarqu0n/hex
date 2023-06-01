#!/usr/bin/env python3
import hexapod_env
import os
import gym
from stable_baselines3 import PPO,DQN
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks  import CheckpointCallback
import rospy
import rospkg

def main():
    train_name = "model1"
    n_steps = rospy.get_param("/nsteps")
    n_epochs = rospy.get_param("/nepisodes")

    package = "hex_training"
    path = rospkg.RosPack().get_path(package)

    save_path = os.path.join(path,"training_results",train_name)
    log_path = os.path.join(path ,"logs",train_name)


    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=save_path,
                                                name_prefix=train_name)
    environment_name = 'Hexapod-v0'
    env = gym.make(environment_name)
    env = DummyVecEnv([lambda: env])
    rospy.logdebug("Learning...")
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path, n_steps=n_steps, n_epochs=n_epochs)
    model.learn(total_timesteps=n_epochs, log_interval=1, callback=checkpoint_callback)
    model.save(save_path + "/ppo_hexapod")




if __name__ == '__main__':
    rospy.init_node('hexapod_gym', anonymous=True)
    main()