#!/usr/bin/env python3
import hexapod_env_idle
import os
import gym
from stable_baselines3 import PPO,DQN,DDPG
from stable_baselines3.common.vec_env import DummyVecEnv,VecNormalize,VecMonitor
from stable_baselines3.common.callbacks  import CheckpointCallback,EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from std_msgs.msg import Int32
import rospy
import rospkg
from stable_baselines3.common.monitor import Monitor

def main():

    train_name = "model1_idle"
    n_steps = rospy.get_param("/nsteps")
    n_epochs = rospy.get_param("/nepisodes")
    gamma = rospy.get_param("/gamma")
    package = "hex_training"
    path = rospkg.RosPack().get_path(package)


    save_path = os.path.join(path,"training_results",train_name)
    log_path = os.path.join(path ,"logs",train_name)

    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=save_path,
                                                name_prefix=train_name)

    environment_name = 'Hexapod-v0-idle'
    env = gym.make(environment_name)
    env = DummyVecEnv([lambda: env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
    vec_env = VecMonitor(env)

    epoch_pub = rospy.Publisher('/train_stats/epoch', Int32, queue_size=1,latch=True)

    model = PPO('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path, n_steps=n_steps, n_epochs=10,gamma=gamma,ent_coef=0.001)
    # model = DDPG('MlpPolicy', env, verbose=2, tensorboard_log=log_path,gamma=gamma,train_freq=(5,"step"))

    # model = DDPG('MultiInputPolicy', env, verbose=2, tensorboard_log=log_path,gamma=gamma,train_freq=(5,"step"))
    rospy.logdebug("Learning...")

    epoch_pub.publish(1)
    model.learn(total_timesteps=100000, log_interval=1, callback=checkpoint_callback,progress_bar=True,reset_num_timesteps=False,tb_log_name=train_name)
    model.save(save_path + "/idlePPO_hexapod")
    env.reset()


    # model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path, n_steps=10, n_epochs=2,gamma=gamma)



if __name__ == '__main__':
    rospy.init_node('hexapod_gym', anonymous=True)
    main()