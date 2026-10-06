import argparse
import os

import pandas as pd

from config import Config
from env.mobile_robot_env import MobileRobotEnv
from methods.followers import DQNFollower, PredictiveDWAFollower, StandardDWAFollower
from run_all import (
    create_performance_tables,
    create_plots,
    make_trial_seed,
    run_statistical_analysis,
)


PROPOSED_METHOD = "A*+DQN-Biased Shield (Proposed)"
METHODS = ["A*+DWA", "A*+Predictive-DWA", PROPOSED_METHOD]


def parse_csv_ints(text):
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def dqn_model_path(env_idx, train_seed, model_tag=None):
    model_name = "dqn_astar"
    if model_tag:
        model_name = f"{model_name}_{model_tag}"
    return f"out/ortam{env_idx}/training/{model_name}_ortam{env_idx}_seed{train_seed}.pth"


def make_env(env_idx):
    return MobileRobotEnv(
        env_idx=env_idx,
        is_eval=True,
        lidar_noise_std=Config.LIDAR_NOISE_STD,
        dyn_obs_speed_mult=Config.DYNAMIC_OBS_SPEED_MULTIPLIER,
        static_obstacle_density=Config.STATIC_OBSTACLE_DENSITY,
        eval_start_pos_noise=Config.EVAL_START_POS_NOISE,
        eval_start_theta_noise=Config.EVAL_START_THETA_NOISE,
        eval_random_start=Config.EVAL_RANDOM_START,
        eval_start_region_radius=Config.EVAL_START_REGION_RADIUS,
        dyn_obs_phase_jitter_steps=Config.DYNAMIC_OBS_PHASE_JITTER_STEPS,
    )


def result_row(env_idx, method_name, train_seed, eval_seed, trial_seed, res, model_tag=None):
    return {
        "Condition": "main",
        "Environment": f"Ortam {env_idx}",
        "Method": method_name,
        "ModelTag": model_tag or "",
        "TrainSeed": train_seed,
        "EvalSeed": eval_seed,
        "TrialSeed": trial_seed,
        "Outcome": res["Outcome"],
        "Success": 1.0 if res["Success"] else 0.0,
        "Collision": 1.0 if res["Collision"] else 0.0,
        "Timeout": 1.0 if res["Timeout"] else 0.0,
        "PlanningFailure": 1.0 if res.get("PlanningFailure", False) else 0.0,
        "DoorSuccess": 1.0 if res.get("DoorSuccess", False) else 0.0,
        "PathLength": res["PathLength"],
        "Time": res["Time"],
        "Smoothness": res["Smoothness"],
        "ControlEnergy": res["ControlEnergy"],
        "MinDist": res["MinDist"],
        "MeanDist": res["MeanDist"],
        "PlanningTime": res["PlanningTime"],
        "ControlTimeMean": res["ControlTimeMean"],
        "ControlTimeP95": res["ControlTimeP95"],
        "LidarNoise": Config.LIDAR_NOISE_STD,
        "DynObsSpeed": Config.DYNAMIC_OBS_SPEED_MULTIPLIER,
        "StaticDensity": Config.STATIC_OBSTACLE_DENSITY,
        "UseLookAhead": True,
        "UseDoorMask": True,
        "UseLidar": True,
        "NoDenseReward": False,
        "Error": "",
    }


def save_checkpoint(row, metrics_path):
    if os.path.exists(metrics_path):
        df = pd.read_csv(metrics_path)
        df = pd.concat([df, pd.DataFrame([row])], ignore_index=True)
    else:
        df = pd.DataFrame([row])
    if "ModelTag" not in df.columns:
        df["ModelTag"] = ""
    df["ModelTag"] = df["ModelTag"].fillna("").astype(str)

    dedupe_columns = [
        "Condition",
        "Environment",
        "Method",
        "ModelTag",
        "TrainSeed",
        "EvalSeed",
        "TrialSeed",
        "LidarNoise",
        "DynObsSpeed",
        "StaticDensity",
        "UseLookAhead",
        "UseDoorMask",
        "UseLidar",
        "NoDenseReward",
    ]
    available = [col for col in dedupe_columns if col in df.columns]
    df = df.drop_duplicates(subset=available, keep="last")
    sort_columns = [col for col in ["Condition", "Environment", "TrainSeed", "EvalSeed", "Method"] if col in df.columns]
    df = df.sort_values(sort_columns).reset_index(drop=True)
    df.to_csv(metrics_path, index=False)


def evaluate_one(env_idx, method_name, train_seed, eval_seed, model_tag=None):
    trial_seed = make_trial_seed(train_seed, eval_seed, 0)
    env = make_env(env_idx)
    model_path = dqn_model_path(env_idx, train_seed, model_tag=model_tag)
    followers = {
        "A*+DWA": StandardDWAFollower(env, use_rrt=False),
        "A*+Predictive-DWA": PredictiveDWAFollower(env, use_rrt=False),
        PROPOSED_METHOD: DQNFollower(env, model_path=model_path, use_astar=True),
    }
    res = followers[method_name].evaluate(seed=trial_seed)
    return result_row(env_idx, method_name, train_seed, eval_seed, trial_seed, res, model_tag=model_tag)


def main():
    parser = argparse.ArgumentParser(description="Checkpointed evaluation runner.")
    parser.add_argument("--envs", default="1,2,3")
    parser.add_argument("--train-seed", type=int, default=42)
    parser.add_argument("--eval-seed-start", type=int, default=1)
    parser.add_argument("--eval-seed-end", type=int, default=10)
    parser.add_argument("--methods", default=",".join(METHODS))
    parser.add_argument("--model-tag", default="")
    parser.add_argument("--skip-summary", action="store_true")
    args = parser.parse_args()

    envs = parse_csv_ints(args.envs)
    methods = [part.strip() for part in args.methods.split(",") if part.strip()]
    metrics_path = "out/comparison/metrics.csv"
    os.makedirs("out/comparison/tables", exist_ok=True)
    os.makedirs("out/comparison/boxplots", exist_ok=True)
    os.makedirs("paper/figures", exist_ok=True)
    os.makedirs("paper/overleaf_template", exist_ok=True)

    total = len(envs) * len(methods) * (args.eval_seed_end - args.eval_seed_start + 1)
    done = 0
    for env_idx in envs:
        for eval_seed in range(args.eval_seed_start, args.eval_seed_end + 1):
            for method_name in methods:
                done += 1
                print(f"[{done}/{total}] env={env_idx} seed={eval_seed} method={method_name}", flush=True)
                row = evaluate_one(env_idx, method_name, args.train_seed, eval_seed, model_tag=args.model_tag or None)
                save_checkpoint(row, metrics_path)
                print(
                    f"    {row['Outcome']} SR={row['Success']:.0f} CR={row['Collision']:.0f} "
                    f"T={row['Time']:.1f} Smooth={row['Smoothness']:.2f} Energy={row['ControlEnergy']:.2f}",
                    flush=True,
                )

    if not args.skip_summary:
        df = pd.read_csv(metrics_path)
        create_plots(df)
        run_statistical_analysis(df, envs)
        create_performance_tables(df)


if __name__ == "__main__":
    main()
