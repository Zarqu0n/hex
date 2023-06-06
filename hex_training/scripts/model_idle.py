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

    train_name = rospy.get_param("/model_name")
    model_type = rospy.get_param("/model")
    total_timesteps = rospy.get_param("/total_timesteps")

    print("Model Type: ",model_type)
    n_steps = rospy.get_param("/nsteps")
    n_epochs = rospy.get_param("/n_epochs")
    gamma = rospy.get_param("/gamma")
    gae_lambda = rospy.get_param("/gae_lambda")
    clip_range = rospy.get_param("/clip_range")
    clip_range_vf = rospy.get_param("/clip_range_vf")
    ent_coef = rospy.get_param("/ent_coef")
    vf_coef = rospy.get_param("/vf_coef")
    max_grad_norm = rospy.get_param("/max_grad_norm")
    package = "hex_training"
    path = rospkg.RosPack().get_path(package)

    #Eval parameters
    eval_freq = rospy.get_param("/eval_freq")
    eval_episodes = rospy.get_param("/eval_episodes")
    eval_log_path = os.path.join(path ,"logs",train_name,"eval")
    deterministic = rospy.get_param("/deterministic")

    save_path = os.path.join(path,"training_results",train_name)
    log_path = os.path.join(path ,"logs",train_name)

    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=save_path,
                                                name_prefix=train_name)

    environment_name = 'Hexapod-v0-idle'
    env = gym.make(environment_name)
    env = DummyVecEnv([lambda: env])
    # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)


    vec_env = VecMonitor(env)
    if model_type == "PPO":
        model = PPO('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path, n_steps=n_steps,
                     n_epochs=n_epochs,gamma=gamma,ent_coef=ent_coef,vf_coef=vf_coef,
                     max_grad_norm=max_grad_norm,clip_range=clip_range,
                     clip_range_vf=clip_range_vf,gae_lambda=gae_lambda)
        
        print("Training with PPO")

    
        
    # elif model_type == "DDPG":

    # model = DDPG('MlpPolicy', env, verbose=2, tensorboard_log=log_path,gamma=gamma,train_freq=(5,"step"))

    eval_callback = EvalCallback(vec_env, best_model_save_path=save_path, log_path=eval_log_path, 
                                 eval_freq=eval_freq, n_eval_episodes=eval_episodes, deterministic=deterministic, render=False)

    # model = DDPG('MultiInputPolicy', env, verbose=2, tensorboard_log=log_path,gamma=gamma,train_freq=(5,"step"))
    rospy.logdebug("Learning...")

    model.learn(total_timesteps=total_timesteps, log_interval=1, callback=eval_callback,progress_bar=True,reset_num_timesteps=False,tb_log_name=train_name)
    model.save(save_path + "/idlePPO_hexapod")
    env.reset()


    # model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path, n_steps=10, n_epochs=2,gamma=gamma)



if __name__ == '__main__':
    rospy.init_node('hexapod_gym', anonymous=True)
    main()