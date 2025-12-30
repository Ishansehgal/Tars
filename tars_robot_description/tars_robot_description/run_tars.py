#!/usr/bin/env python3
import gymnasium as gym
from stable_baselines3 import PPO
from tars_robot_description.tars_env import TarsEnv
import time

def main():
    # Create Environment
    env = TarsEnv()
    
    # Load Model
    # Try looking for interrupted model first, then finished model
    model_path = "tars_ppo_model_interrupted"
    try:
        model = PPO.load(model_path)
        print(f"Loaded {model_path}")
    except:
        try:
            model = PPO.load("tars_ppo_model")
            print("Loaded tars_ppo_model")
        except:
            print("No model found! Did you save it?")
            return

    obs, _ = env.reset()
    
    print("Running Trained Policy... Press Ctrl+C to stop.")
    try:
        while True:
            # Predict action
            action, _states = model.predict(obs, deterministic=True)
            
            # Step env
            obs, reward, terminated, truncated, info = env.step(action)
            
            if terminated or truncated:
                print("Episode Finished. Resetting...")
                obs, _ = env.reset()
                
    except KeyboardInterrupt:
        print("Stopping...")

if __name__ == '__main__':
    main()
