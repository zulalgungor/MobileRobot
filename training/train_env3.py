import os
import matplotlib
matplotlib.use('Agg')
from methods.dqn.dqn_trainer import train_dqn
from config import Config

print("He-Man (Env-3) Eğitimlere Başlıyor...")
for seed in Config.TRAIN_SEEDS:
    print(f"\n--- Seed: {seed} için eğitim başlatılıyor ---")
    train_dqn(env_idx=3, num_episodes=1500, output_dir="out/ortam3/training/", use_astar=True, seed=seed)
    train_dqn(env_idx=3, num_episodes=1500, output_dir="out/ortam3/training/", use_astar=False, seed=seed)
print("He-Man (Env-3) Eğitimleri Tamamlandı.")
