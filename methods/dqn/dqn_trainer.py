import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import torch
from config import Config
import random
from env.mobile_robot_env import MobileRobotEnv
from methods.dqn.dqn_agent import DQNAgent

def curriculum_scale(ep, total_episodes):
    if not Config.USE_TRAIN_CURRICULUM:
        return 1.0
    warmup = max(1, int(total_episodes * Config.TRAIN_CURRICULUM_WARMUP_FRAC))
    ramp = max(1, int(total_episodes * Config.TRAIN_CURRICULUM_RAMP_FRAC))
    if ep <= warmup:
        return 0.0
    return min(1.0, (ep - warmup) / ramp)

def train_dqn(env_idx=1, num_episodes=Config.NUM_EPISODES, output_dir="out/ortam1/training/",
              use_astar=True, seed=None, env_kwargs=None, model_tag=None):
    os.makedirs(output_dir, exist_ok=True)
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)
        
    env_kwargs = dict(env_kwargs or {})
    target_train_eval_start_prob = env_kwargs.pop("train_eval_start_prob", Config.TRAIN_EVAL_START_PROB)
    target_train_dyn_obs_phase_jitter_steps = env_kwargs.pop(
        "train_dyn_obs_phase_jitter_steps",
        Config.TRAIN_DYN_OBS_PHASE_JITTER_STEPS,
    )
    if Config.USE_TRAIN_CURRICULUM:
        env_kwargs.setdefault("train_eval_start_prob", 0.0)
        env_kwargs.setdefault("train_dyn_obs_phase_jitter_steps", 0)
    else:
        env_kwargs.setdefault("train_eval_start_prob", target_train_eval_start_prob)
        env_kwargs.setdefault("train_dyn_obs_phase_jitter_steps", target_train_dyn_obs_phase_jitter_steps)
    env_kwargs.setdefault("eval_start_region_radius", Config.EVAL_START_REGION_RADIUS)
    env = MobileRobotEnv(env_idx=env_idx, is_eval=False, **env_kwargs)
    env.use_astar = use_astar
    agent = DQNAgent(
        state_dim=24,
        action_dim=7,
        dueling=Config.USE_ENHANCED_DQN,
        prioritized_replay=Config.USE_ENHANCED_DQN,
        n_step=Config.N_STEP if Config.USE_ENHANCED_DQN else 1,
        reward_clip=Config.REWARD_CLIP if Config.USE_ENHANCED_DQN else None,
    )
    
    epsilon = Config.EPSILON_START
    eps_min = Config.EPSILON_MIN
    eps_decay = Config.EPSILON_DECAY
    
    ep_returns = []
    ep_success = []
    ep_collisions = []
    best_recent_sr = -1.0
    best_recent_cr = 1.0
    
    seed_str = f"_seed{seed}" if seed is not None else ""
    model_name = "dqn_astar" if use_astar else "dqn_only"
    if model_tag:
        model_name = f"{model_name}_{model_tag}"
    print(f"--- Ortam-{env_idx} {model_name} Eğitimi Başlatıldı ({num_episodes} Bölüm, Seed: {seed}) ---")
    
    for ep in range(1, num_episodes + 1):
        scale = curriculum_scale(ep, num_episodes)
        env.train_eval_start_prob = target_train_eval_start_prob * scale
        env.train_dyn_obs_phase_jitter_steps = int(round(target_train_dyn_obs_phase_jitter_steps * scale))

        state, _ = env.reset(seed=seed if ep == 1 else None)
        total_reward = 0.0
        success = False
        collision = False
        
        while True:
            action_idx = agent.act(state, epsilon)
            action_w = agent.get_w_action(action_idx)
            
            next_state, reward, terminated, truncated, info = env.step(action_w)
            done = terminated or truncated
            
            agent.step(state, action_idx, reward, next_state, float(terminated))
            agent.learn()
            
            state = next_state
            total_reward += reward
            
            if done:
                success = info.get("success", False)
                collision = info.get("collision", False)
                break
                
        epsilon = max(eps_min, epsilon * eps_decay)
        
        ep_returns.append(total_reward)
        ep_success.append(1.0 if success else 0.0)
        ep_collisions.append(1.0 if collision else 0.0)
        
        if ep % 20 == 0:
            recent_sr = np.mean(ep_success[-20:])
            recent_cr = np.mean(ep_collisions[-20:])
            if recent_sr > best_recent_sr or (recent_sr == best_recent_sr and recent_cr < best_recent_cr):
                best_recent_sr = recent_sr
                best_recent_cr = recent_cr
                best_model_path = os.path.join(output_dir, f"{model_name}_best_ortam{env_idx}{seed_str}.pth")
                agent.save(best_model_path)
                print(f"Best checkpoint saved: {best_model_path} (SR={recent_sr:.2f}, CR={recent_cr:.2f})")
            print(
                f"Bölüm {ep}/{num_episodes} | Ödül: {total_reward:.1f} | "
                f"Başarı Oranı: %{recent_sr*100:.1f} | Çarpışma: %{recent_cr*100:.1f} | "
                f"Epsilon: {epsilon:.2f} | EvalStart: {env.train_eval_start_prob:.2f} | "
                f"DynJitter: {env.train_dyn_obs_phase_jitter_steps}"
            )

    model_path = os.path.join(output_dir, f"{model_name}_ortam{env_idx}{seed_str}.pth")
    agent.save(model_path)
    print(f"Model kaydedildi: {model_path}")
    
    plt.figure(figsize=(15, 5))
    
    # Reward Subplot
    plt.subplot(1, 3, 1)
    plt.plot(ep_returns, alpha=0.3, color='blue')
    if len(ep_returns) >= 20:
        ma = np.convolve(ep_returns, np.ones(20)/20, mode='valid')
        plt.plot(np.arange(19, len(ep_returns)), ma, color='blue', linewidth=2)
    plt.xlabel("Episode")
    plt.ylabel("Total Reward")
    plt.title("Reward Convergence")
    plt.grid(True)
    
    # Success Subplot
    plt.subplot(1, 3, 2)
    plt.plot(ep_success, alpha=0.1, color='green')
    if len(ep_success) >= 20:
        ma_succ = np.convolve(ep_success, np.ones(20)/20, mode='valid')
        plt.plot(np.arange(19, len(ep_success)), ma_succ, color='green', linewidth=2)
    plt.xlabel("Episode")
    plt.ylabel("Success Rate")
    plt.title("Moving Success Rate")
    plt.grid(True)
    
    # Collision Subplot
    plt.subplot(1, 3, 3)
    plt.plot(ep_collisions, alpha=0.1, color='red')
    if len(ep_collisions) >= 20:
        ma_coll = np.convolve(ep_collisions, np.ones(20)/20, mode='valid')
        plt.plot(np.arange(19, len(ep_collisions)), ma_coll, color='red', linewidth=2)
    plt.xlabel("Episode")
    plt.ylabel("Collision Rate")
    plt.title("Moving Collision Rate")
    plt.grid(True)
    
    plt.suptitle(f"Ortam-{env_idx} {model_name} (Seed: {seed}) Eğitim Yakınsama Grafikleri")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f"{model_name}_convergence_ortam{env_idx}{seed_str}.png"))
    plt.close()
    
    return agent
