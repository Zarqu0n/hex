#!/usr/bin/env python3
import hexapod_envs
import os
from functions import *
import gymnasium as gym
from stable_baselines3 import PPO,DQN,DDPG,SAC,A2C
from stable_baselines3.common.vec_env import DummyVecEnv,VecNormalize,VecMonitor
from stable_baselines3.common.callbacks  import EvalCallback
import rospy
import rospkg
import shutil
def main():

    model_name = rospy.get_param("/model_name")
    model_type = rospy.get_param("/model")
    total_timesteps = rospy.get_param("/total_timesteps")
    learning_rate = rospy.get_param("/learning_rate")
    training_step = rospy.get_param("/training_step")

    print("Model Type: ",model_type)
    environment_name = rospy.get_param("/env")
    package = "hex_training"
    path = rospkg.RosPack().get_path(package)
    #Eval parameters
    train_name = model_name + "_" + model_type
    eval_freq = rospy.get_param("/eval_freq")
    eval_episodes = rospy.get_param("/eval_episodes")
    eval_log_path = os.path.join(path ,"training_results",environment_name,train_name,"logs","eval")
    deterministic = rospy.get_param("/deterministic")
    train_file_name = rospy.get_param("~file_name")
    save_path = os.path.join(path,"training_results",environment_name,train_name)
    log_path = os.path.join(path ,"training_results",environment_name,train_name,"logs")
    log_name = train_name

    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(eval_log_path):
        os.makedirs(eval_log_path)
    if not os.path.exists(log_path):
        os.makedirs(log_path)

    continue_train,current_timestep = find_latest_model(directory=save_path,prefix=train_name)

    if continue_train == None:
        shutil.copy(os.path.join(path,"config",f"{train_file_name}.yaml"),os.path.join(path,save_path))

    env = gym.make(environment_name)
    env = DummyVecEnv([lambda: env])
    # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
    # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)


    vec_env = VecMonitor(env)
    if model_type == "PPO":
        # PPO parameters
        n_steps = rospy.get_param("/PPO/nsteps")
        n_epochs = rospy.get_param("/PPO/n_epochs")
        gamma = rospy.get_param("/PPO/gamma")
        gae_lambda = rospy.get_param("/PPO/gae_lambda")
        clip_range = rospy.get_param("/PPO/clip_range")
        clip_range_vf = rospy.get_param("/PPO/clip_range_vf")
        ent_coef = rospy.get_param("/PPO/ent_coef")
        vf_coef = rospy.get_param("/PPO/vf_coef")
        max_grad_norm = rospy.get_param("/PPO/max_grad_norm")


        model = PPO('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path, n_steps=n_steps,
                    n_epochs=n_epochs,gamma=gamma,ent_coef=ent_coef,vf_coef=vf_coef,
                    max_grad_norm=max_grad_norm,clip_range=clip_range,
                    clip_range_vf=clip_range_vf,gae_lambda=gae_lambda,batch_size=n_steps)
        
        print("Training with PPO")    
        
    elif model_type == "DDPG":

        # DDPG parameters
        tau = rospy.get_param("/tau")
        gamma = rospy.get_param("/gamma")
        train_freq_number = rospy.get_param("/train_freq_number")
        train_freq_unit = rospy.get_param("/train_freq_unit")
        learning_starts = rospy.get_param("/learning_starts")
        batch_size = rospy.get_param("/batch_size")

        model = DDPG('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path,gamma=gamma,
                     learning_rate=learning_rate,batch_size=batch_size,tau=tau,
                     train_freq=(train_freq_number,train_freq_unit),learning_starts=learning_starts)

        print("Training with DDPG")


    elif model_type == "A2C":
        a2c_gamma = rospy.get_param("/a2c/gamma")
        a2c_n_steps = rospy.get_param("/a2c/n_steps")
        a2c_vf_coef = rospy.get_param("/a2c/vf_coef")
        a2c_ent_coef = rospy.get_param("/a2c/ent_coef")
        a2c_max_grad_norm = rospy.get_param("/a2c/max_grad_norm")
        a2c_learning_rate = rospy.get_param("/a2c/learning_rate")
        a2c_max_grad_norm = rospy.get_param("/a2c/max_grad_norm")
        a2c_gae_lambda = rospy.get_param("/a2c/gae_lambda")
        a2c_rms_prop_eps = rospy.get_param("/a2c/rms_prop_eps") 

        model = A2C('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path,gamma=a2c_gamma,
                    learning_rate=a2c_learning_rate,max_grad_norm=a2c_max_grad_norm,n_steps=a2c_n_steps,
                    vf_coef=a2c_vf_coef,ent_coef=a2c_ent_coef,gae_lambda=a2c_gae_lambda)
    
        print("Training with A2C")

    elif model_type == "SAC":
        model = SAC('MultiInputPolicy', vec_env, verbose=2, tensorboard_log=log_path,learning_rate=learning_rate)
        print("Training with SAC")
    
    eval_callback = EvalCallback(vec_env, best_model_save_path=save_path, log_path=eval_log_path, 
                                 eval_freq=eval_freq, n_eval_episodes=eval_episodes, deterministic=deterministic, render=False)

    rospy.logdebug("Learning...")

    if continue_train != None:
        model = model.load(save_path+"/"+continue_train,env=vec_env, tensorboard_log=log_path+"/"+log_name )
        total_timesteps -= current_timestep
        print(f"-----------------------------------Continue train with {continue_train}")

    for i in range(int(total_timesteps/training_step)):

        step_name = train_name + "_" + str((i+1)*training_step)
        model.set_env(vec_env)
        model.learn(total_timesteps=training_step, log_interval=1, callback=eval_callback,progress_bar=True,reset_num_timesteps=False)
        env.close()
        model.save(save_path+"/"+step_name )

        if i < training_step:
            model.load(save_path+"/"+step_name,env=vec_env, tensorboard_log=log_path+"/"+log_name )


if __name__ == '__main__':
    rospy.init_node('hexapod_gym', anonymous=True)
    main()