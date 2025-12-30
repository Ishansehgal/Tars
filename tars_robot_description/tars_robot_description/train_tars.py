#!/usr/bin/env python3
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from tars_robot_description.tars_env import TarsEnv
import os

def main():
    # Create Environment
    env = TarsEnv()
    
    # Check Environment
    # check_env(env) # Optional, can fail due to loose bounds but good for debugging
    
    # Initialize PPO Agent (Use CUDA if available)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./tars_tensorboard/", device="cuda")
    
    print("Starting Training on GPU...")
    try:
        # Train for 100k timesteps
        model.learn(total_timesteps=100000)
        
        print("Training Finished. Saving Model...")
        model.save("tars_ppo_model")
        
    except KeyboardInterrupt:
        print("Training Interrupted. Saving...")
        model.save("tars_ppo_model_interrupted")

if __name__ == '__main__':
    main()
