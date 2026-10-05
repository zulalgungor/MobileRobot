import os
import matplotlib
matplotlib.use('Agg')
from methods.dqn.dqn_trainer import train_dqn
from config import Config

print("Edi (Env-1) Eğitimlere Başlıyor...")
for seed in Config.TRAIN_SEEDS:
    print(f"\n--- Seed: {seed} için eğitim başlatılıyor ---")
    train_dqn(env_idx=1, num_episodes=1000, output_dir="out/ortam1/training/", use_astar=True, seed=seed)
    train_dqn(env_idx=1, num_episodes=1000, output_dir="out/ortam1/training/", use_astar=False, seed=seed)
print("Edi (Env-1) Eğitimleri Tamamlandı.")
