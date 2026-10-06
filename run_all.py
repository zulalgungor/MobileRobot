import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from env.mobile_robot_env import MobileRobotEnv
from methods.dqn.dqn_trainer import train_dqn
from methods.followers import DQNFollower, PredictiveDWAFollower, StandardDWAFollower
from methods.visualizer import create_animation_gif, save_static_trajectory
from config import Config

try:
    from scipy.stats import kruskal, mannwhitneyu
    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False


PROPOSED_METHOD = "A*+DQN-Biased Shield (Proposed)"
METHODS = ["A*+DWA", "A*+Predictive-DWA", PROPOSED_METHOD]
BASELINES = ["A*+DWA", "A*+Predictive-DWA"]
SUCCESS_ONLY_METRICS = [
    "PathLength",
    "Time",
    "Smoothness",
    "ControlEnergy",
    "MinDist",
    "MeanDist",
    "ControlTimeMean",
    "ControlTimeP95",
]


def parse_envs(text):
    envs = [int(part.strip()) for part in text.split(",") if part.strip()]
    return envs if envs else [1, 2, 3]


def parse_int_list(text):
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_float_list(text):
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_train_episodes(text):
    mapping = {1: 1000, 2: 1500, 3: 1500}
    if not text:
        return mapping

    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        env_id, episode_count = chunk.split(":")
        mapping[int(env_id)] = int(episode_count)
    return mapping


def make_trial_seed(train_seed, eval_seed, condition_idx=0):
    return int((condition_idx + 1) * 1_000_000 + train_seed * 1_000 + eval_seed)


def build_experiment_conditions(args):
    main = {
        "name": "main",
        "lidar_noise": args.lidar_noise,
        "dyn_obs_speed": args.dyn_obs_speed,
        "static_density": args.static_density,
        "use_look_ahead": not args.no_lookahead,
        "use_door_mask": not args.no_doormask,
        "use_lidar": not args.no_lidar,
        "ablation_no_dense_reward": args.no_dense_reward,
        "methods": METHODS,
    }

    conditions = []
    if args.experiment_suite in ("main", "all"):
        conditions.append(main)

    if args.experiment_suite in ("robustness", "all"):
        for noise in parse_float_list(args.robustness_lidar_noise):
            cond = dict(main)
            cond.update({"name": f"robust_lidar_{noise:g}", "lidar_noise": noise})
            conditions.append(cond)
        for speed in parse_float_list(args.robustness_dyn_speed):
            cond = dict(main)
            cond.update({"name": f"robust_dynspeed_{speed:g}", "dyn_obs_speed": speed})
            conditions.append(cond)
        for density in parse_float_list(args.robustness_static_density):
            cond = dict(main)
            cond.update({"name": f"robust_staticdensity_{density:g}", "static_density": density})
            conditions.append(cond)

    if args.experiment_suite in ("ablation", "all"):
        ablations = [
            ("ablation_no_lookahead", {"use_look_ahead": False}),
            ("ablation_no_doormask", {"use_door_mask": False}),
            ("ablation_no_lidar", {"use_lidar": False}),
            ("ablation_no_dense_reward", {"ablation_no_dense_reward": True}),
        ]
        for name, overrides in ablations:
            cond = dict(main)
            cond.update(overrides)
            cond.update({"name": name, "methods": [PROPOSED_METHOD]})
            conditions.append(cond)

    return conditions if conditions else [main]


def model_tag_for_condition(condition):
    name = condition["name"]
    return name if name.startswith("ablation_") else None


def dqn_model_path(env_idx, train_seed, model_tag=None):
    model_name = "dqn_astar"
    if model_tag:
        model_name = f"{model_name}_{model_tag}"
    return f"out/ortam{env_idx}/training/{model_name}_ortam{env_idx}_seed{train_seed}.pth"


def env_kwargs_from_condition(condition, args, include_eval_params=True):
    kwargs = {
        "lidar_noise_std": condition["lidar_noise"],
        "dyn_obs_speed_mult": condition["dyn_obs_speed"],
        "use_look_ahead": condition["use_look_ahead"],
        "use_door_mask": condition["use_door_mask"],
        "use_lidar": condition["use_lidar"],
        "ablation_no_dense_reward": condition["ablation_no_dense_reward"],
        "static_obstacle_density": condition["static_density"],
    }
    if include_eval_params:
        kwargs.update({
            "eval_start_pos_noise": args.eval_start_pos_noise,
            "eval_start_theta_noise": args.eval_start_theta_noise,
            "eval_random_start": not args.fixed_eval_start,
            "eval_start_region_radius": args.eval_start_region_radius,
            "dyn_obs_phase_jitter_steps": args.dyn_phase_jitter_steps,
        })
    return kwargs


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Run Q1-style robot navigation experiments.")
    parser.add_argument("--envs", default="1,2,3", help="Comma-separated environment ids, e.g. 1,2,3")
    parser.add_argument("--num-seeds", type=int, default=Config.EVAL_SEEDS_PER_TRAIN_SEED, help="Evaluation seeds per method/environment/train seed")
    parser.add_argument("--eval-seed-start", type=int, default=1, help="First evaluation seed to run; use with --append-results to extend existing results")
    parser.add_argument("--append-results", action="store_true", help="Append new run-level metrics to the metrics file and regenerate summaries from the combined data")
    parser.add_argument("--resume", action="store_true", help="Resume an interrupted evaluation from the checkpoint/final metrics file; use only with the same experiment settings")
    parser.add_argument("--checkpoint-every", type=int, default=1, help="Save an evaluation checkpoint after this many newly completed method-runs (default: 1)")
    parser.add_argument("--metrics-path", default="out/comparison/metrics.csv", help="Path to save evaluation metrics")
    parser.add_argument("--gif-seeds", type=int, default=0, help="Create GIF/static trajectory files for evaluation seeds <= this value; 0 disables visualization (default: 0)")
    parser.add_argument("--skip-training", action="store_true", help="Use existing DQN models only")
    parser.add_argument("--train-seeds", default=",".join(map(str, Config.TRAIN_SEEDS)), help="Comma-separated independent DQN training seeds")
    parser.add_argument("--eval-model-seeds", default="", help="Comma-separated model seeds to evaluate; defaults to --train-seeds")
    parser.add_argument(
        "--train-episodes",
        default="1:1000,2:1500,3:1500",
        help="Per-environment training budget, e.g. 1:1000,2:1500,3:1500",
    )
    parser.add_argument("--lidar-noise", type=float, default=Config.LIDAR_NOISE_STD, help="Standard deviation of LiDAR noise")
    parser.add_argument("--dyn-obs-speed", type=float, default=Config.DYNAMIC_OBS_SPEED_MULTIPLIER, help="Speed multiplier for dynamic obstacles")
    parser.add_argument("--static-density", type=float, default=Config.STATIC_OBSTACLE_DENSITY, help="Extra static obstacle density for map-density robustness")
    parser.add_argument("--eval-start-pos-noise", type=float, default=Config.EVAL_START_POS_NOISE, help="Evaluation start position Gaussian noise (m)")
    parser.add_argument("--eval-start-theta-noise", type=float, default=Config.EVAL_START_THETA_NOISE, help="Evaluation start heading Gaussian noise (rad)")
    parser.add_argument("--fixed-eval-start", action="store_true", help="Disable random evaluation start-region sampling")
    parser.add_argument("--eval-start-region-radius", type=float, default=Config.EVAL_START_REGION_RADIUS, help="Radius around the nominal start for random evaluation starts (m)")
    parser.add_argument("--dyn-phase-jitter-steps", type=int, default=Config.DYNAMIC_OBS_PHASE_JITTER_STEPS, help="Random phase jitter steps for dynamic obstacles during evaluation")
    parser.add_argument("--no-lookahead", action="store_true", help="Ablation: Disable A* look-ahead")
    parser.add_argument("--no-doormask", action="store_true", help="Ablation: Disable door mask in A*")
    parser.add_argument("--no-lidar", action="store_true", help="Ablation: Disable LiDAR observations")
    parser.add_argument("--no-dense-reward", action="store_true", help="Ablation: Disable dense shaping reward")
    parser.add_argument("--experiment-suite", choices=["main", "robustness", "ablation", "all"], default="main", help="Which experiment conditions to run")
    parser.add_argument("--robustness-lidar-noise", default=",".join(map(str, Config.ROBUSTNESS_LIDAR_NOISE)), help="LiDAR noise levels for robustness suite")
    parser.add_argument("--robustness-dyn-speed", default=",".join(map(str, Config.ROBUSTNESS_DYN_SPEED)), help="Dynamic obstacle speed multipliers for robustness suite")
    parser.add_argument("--robustness-static-density", default=",".join(map(str, Config.ROBUSTNESS_STATIC_DENSITY)), help="Static obstacle density levels for robustness suite")
    parser.add_argument("--eval-model-seed", type=int, default=None, help="Backward-compatible alias for evaluating a single model seed")
    return parser


def ensure_output_dirs(environments):
    folders = [
        "out/comparison/boxplots",
        "out/comparison/tables",
        "paper/figures",
        "paper/overleaf_template",
    ]
    for env_idx in environments:
        folders.extend([
            f"out/ortam{env_idx}/training",
            f"out/ortam{env_idx}/evaluation",
            f"out/ortam{env_idx}/animations",
        ])
    for folder in folders:
        os.makedirs(folder, exist_ok=True)


def mean_std_ci(values):
    arr = pd.to_numeric(pd.Series(values), errors="coerce").dropna().to_numpy(dtype=float)
    if arr.size == 0:
        return np.nan, np.nan, np.nan, 0
    mean = float(np.mean(arr))
    std = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
    ci95 = float(1.96 * std / np.sqrt(arr.size)) if arr.size > 1 else 0.0
    return mean, std, ci95, int(arr.size)


def fmt_mean_std_ci(values):
    mean, std, ci95, n = mean_std_ci(values)
    if n == 0:
        return "NA"
    return f"{mean:.3f} +/- {std:.3f} [{mean - ci95:.3f}, {mean + ci95:.3f}]"


def bootstrap_ci_interval(values, n_bootstraps=1000, alpha=0.05):
    arr = pd.to_numeric(pd.Series(values), errors="coerce").dropna().to_numpy(dtype=float)
    n = len(arr)
    if n < 2:
        m = float(np.mean(arr)) if n == 1 else 0.0
        return m, m
    
    boot_means = np.random.choice(arr, size=(n_bootstraps, n), replace=True).mean(axis=1)
    lower = np.percentile(boot_means, 100 * (alpha / 2))
    upper = np.percentile(boot_means, 100 * (1 - alpha / 2))
    return float(lower), float(upper)


def cliffs_delta(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.size == 0 or y.size == 0:
        return np.nan
    diffs = x[:, None] - y[None, :]
    greater = np.sum(diffs > 0)
    lower = np.sum(diffs < 0)
    return float((greater - lower) / (x.size * y.size))


def holm_bonferroni(p_values, alpha=0.05):
    p_values = np.asarray(p_values, dtype=float)
    adjusted = np.empty_like(p_values)
    order = np.argsort(p_values)
    sorted_p = p_values[order]
    m = len(sorted_p)
    sorted_adjusted = np.maximum.accumulate((m - np.arange(m)) * sorted_p)
    sorted_adjusted = np.minimum(sorted_adjusted, 1.0)
    adjusted[order] = sorted_adjusted
    return adjusted <= alpha, adjusted



RESULT_KEY_COLUMNS = [
    "Condition",
    "Environment",
    "Method",
    "TrainSeed",
    "EvalSeed",
    "LidarNoise",
    "DynObsSpeed",
    "StaticDensity",
    "UseLookAhead",
    "UseDoorMask",
    "UseLidar",
    "NoDenseReward",
]


def _normalize_key_value(value):
    if pd.isna(value):
        return None
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return round(float(value), 12)
    text = str(value).strip()
    if text.lower() in {"true", "false"}:
        return text.lower() == "true"
    try:
        number = float(text)
        if number.is_integer():
            return int(number)
        return round(number, 12)
    except ValueError:
        return text


def result_key_from_row(row):
    return tuple(_normalize_key_value(row.get(col)) for col in RESULT_KEY_COLUMNS)


def expected_result_key(condition, env_idx, method_name, train_seed, eval_seed):
    row = {
        "Condition": condition["name"],
        "Environment": f"Ortam {env_idx}",
        "Method": method_name,
        "TrainSeed": train_seed,
        "EvalSeed": eval_seed,
        "LidarNoise": condition["lidar_noise"],
        "DynObsSpeed": condition["dyn_obs_speed"],
        "StaticDensity": condition["static_density"],
        "UseLookAhead": condition["use_look_ahead"],
        "UseDoorMask": condition["use_door_mask"],
        "UseLidar": condition["use_lidar"],
        "NoDenseReward": condition["ablation_no_dense_reward"],
    }
    return result_key_from_row(row)


def dedupe_and_sort_results(df):
    if df.empty:
        return df
    dedupe_columns = [col for col in RESULT_KEY_COLUMNS if col in df.columns]
    if dedupe_columns:
        df = df.drop_duplicates(subset=dedupe_columns, keep="last")
    sort_columns = [
        col for col in ["Condition", "Environment", "TrainSeed", "EvalSeed", "Method"]
        if col in df.columns
    ]
    if sort_columns:
        df = df.sort_values(sort_columns).reset_index(drop=True)
    return df


def checkpoint_path_for(metrics_path):
    root, ext = os.path.splitext(metrics_path)
    if not ext:
        ext = ".csv"
    return f"{root}_checkpoint{ext}"


def atomic_write_csv(df, path):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    df.to_csv(tmp_path, index=False)
    os.replace(tmp_path, path)


def save_evaluation_checkpoint(results_list, checkpoint_path):
    if not results_list:
        return
    checkpoint_df = dedupe_and_sort_results(pd.DataFrame(results_list))
    atomic_write_csv(checkpoint_df, checkpoint_path)


def load_resume_rows(metrics_path, checkpoint_path):
    frames = []
    for path in (metrics_path, checkpoint_path):
        if os.path.exists(path):
            try:
                frame = pd.read_csv(path)
                if not frame.empty:
                    frames.append(frame)
            except (pd.errors.EmptyDataError, OSError) as exc:
                print(f"WARNING: Could not read resume file {path}: {exc}")
    if not frames:
        return pd.DataFrame()
    return dedupe_and_sort_results(pd.concat(frames, ignore_index=True, sort=False))

def create_plots(df):
    print("\n=== Step 3: Creating figures ===")
    if "ModelTag" in df.columns:
        df = df[df["ModelTag"].fillna("") == ""].copy()

    FIG_DPI = 600
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 9,
        "axes.labelsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 9,
        "axes.titlesize": 9,
    })
    sns.set_theme(style="ticks", context="paper", font_scale=1.0)

    my_palette = sns.color_palette("tab10")
    plot_source = df[df["Condition"] == "main"] if "Condition" in df.columns and "main" in set(df["Condition"]) else df

    def save_current_figure(out_name):
        plt.tight_layout()
        plt.savefig(f"out/comparison/boxplots/{out_name}.png", dpi=FIG_DPI, bbox_inches='tight')
        plt.savefig(f"paper/figures/{out_name}.png", dpi=FIG_DPI, bbox_inches='tight')
        plt.close()

    # Success rate plot (no legend, no title)
    success_df = plot_source.groupby(["Environment", "Method"], as_index=False)["Success"].mean()
    success_df["Success"] *= 100.0
    plt.figure(figsize=(8.2, 5.0))
    ax = sns.barplot(
        x="Environment",
        y="Success",
        hue="Method",
        data=success_df,
        palette=my_palette,
        edgecolor=".2",
        capsize=0.05,
        errcolor=".2",
        errwidth=1.2,
    )
    plt.ylabel("Success rate (%)")
    plt.xlabel("Environment")
    plt.ylim(0, 105)
    legend = ax.get_legend()
    if legend is not None:
        legend.remove()
    sns.despine()
    ax.yaxis.grid(True, linestyle='--', alpha=0.30)
    save_current_figure("success_barplot")

    # Outcome distribution (no legend, no title)
    outcome_df = plot_source.groupby(["Environment", "Method", "Outcome"], as_index=False).size()
    outcome_plot = sns.catplot(
        x="Method",
        y="size",
        hue="Outcome",
        col="Environment",
        data=outcome_df,
        kind="bar",
        palette="muted",
        height=4.6,
        aspect=1.05,
        sharey=True,
        edgecolor=".2",
        legend=False,
    )
    outcome_plot.set_axis_labels("Method", "Run count")
    outcome_plot.set_xticklabels(rotation=45)
    for ax_facet in outcome_plot.axes.flat:
        ax_facet.yaxis.grid(True, linestyle='--', alpha=0.30)
        leg = ax_facet.get_legend()
        if leg is not None:
            leg.remove()
    outcome_plot.tight_layout()
    outcome_plot.savefig("out/comparison/boxplots/outcome_distribution.png", dpi=FIG_DPI, bbox_inches='tight')
    outcome_plot.savefig("paper/figures/outcome_distribution.png", dpi=FIG_DPI, bbox_inches='tight')
    plt.close(outcome_plot.fig)

    metric_titles = {
        "PathLength": "Path length (m)",
        "Time": "Completion time (s)",
        "Smoothness": "Control smoothness",
        "ControlEnergy": "Control energy",
        "MinDist": "Minimum distance to obstacles (m)",
        "MeanDist": "Mean distance to obstacles (m)",
        "ControlTimeMean": "Mean control latency (s)",
        "ControlTimeP95": "P95 control latency (s)",
    }

    successful_df = plot_source[plot_source["Success"] == 1.0]
    for metric, y_label in metric_titles.items():
        plot_df = successful_df if len(successful_df) > 0 else df
        if metric not in plot_df or plot_df[metric].dropna().empty:
            continue
        plt.figure(figsize=(8.2, 5.0))
        ax = sns.boxplot(
            x="Environment",
            y=metric,
            hue="Method",
            data=plot_df,
            palette=my_palette,
            showfliers=False,
            linewidth=1.2,
            width=0.6,
        )
        plt.ylabel(y_label)
        plt.xlabel("Environment")
        legend = ax.get_legend()
        if legend is not None:
            legend.remove()
        sns.despine()
        ax.yaxis.grid(True, linestyle='--', alpha=0.30)
        save_current_figure(f"{metric.lower()}_boxplot")

    if "PathLength" in successful_df and not successful_df["PathLength"].dropna().empty:
        env_order = sorted(successful_df["Environment"].unique())
        method_label_map = {
            PROPOSED_METHOD: "DQN-Shield",
            "A*+DWA": "DWA",
            "A*+Predictive-DWA": "Predictive-DWA",
        }
        fig, axes = plt.subplots(1, len(env_order), figsize=(4.5 * len(env_order), 4.8), sharey=False, facecolor="white")
        if len(env_order) == 1:
            axes = [axes]
        for ax, env_label in zip(axes, env_order):
            env_df = successful_df[successful_df["Environment"] == env_label].copy()
            env_df["MethodShort"] = env_df["Method"].map(method_label_map).fillna(env_df["Method"])
            sns.boxplot(
                x="MethodShort",
                y="PathLength",
                data=env_df,
                ax=ax,
                palette=my_palette,
                showfliers=False,
                linewidth=1.2,
                width=0.55,
            )
            sns.stripplot(
                x="MethodShort",
                y="PathLength",
                data=env_df,
                ax=ax,
                color="black",
                alpha=0.10,
                size=1.4,
                jitter=0.22,
            )
            ax.set_xlabel("")
            ax.set_ylabel("Path length (m)" if ax is axes[0] else "")
            ax.tick_params(axis="x", rotation=0)
            ax.grid(True, axis="y", linestyle="--", alpha=0.25)
            sns.despine(ax=ax)
        fig.tight_layout()
        fig.savefig("out/comparison/boxplots/pathlength_env_panels.png", dpi=FIG_DPI, bbox_inches="tight")
        fig.savefig("paper/figures/pathlength_env_panels.png", dpi=FIG_DPI, bbox_inches="tight")
        plt.close(fig)

    print("Figures saved under out/comparison/boxplots and paper/figures.")


def create_performance_tables(df):
    print("\n=== Step 5: Creating performance tables ===")
    if "ModelTag" in df.columns:
        df = df[df["ModelTag"].fillna("") == ""].copy()
    numeric_rows = []
    formatted_rows = []

    group_cols = ["Environment", "Method"]
    if "Condition" in df.columns:
        group_cols = ["Condition", *group_cols]

    for keys, group in df.groupby(group_cols, sort=False):
        if "Condition" in df.columns:
            condition, environment, method = keys
        else:
            environment, method = keys
            condition = None
        successful = group[group["Success"] == 1.0]
        numeric_row = {
            "Environment": environment,
            "Method": method,
            "N": int(len(group)),
            "SR": float(group["Success"].mean()),
            "CR": float(group["Collision"].mean()),
            "TR": float(group["Timeout"].mean()),
            "DoorSR": float(group["DoorSuccess"].mean()),
            "PFR": float(group["PlanningFailure"].mean()),
        }
        if condition is not None:
            numeric_row = {"Condition": condition, **numeric_row}
        sr_low, sr_high = bootstrap_ci_interval(group["Success"])
        cr_low, cr_high = bootstrap_ci_interval(group["Collision"])
        tr_low, tr_high = bootstrap_ci_interval(group["Timeout"])
        doorsr_low, doorsr_high = bootstrap_ci_interval(group["DoorSuccess"])
        pfr_low, pfr_high = bootstrap_ci_interval(group["PlanningFailure"])

        numeric_row.update({
            "SR_lower": sr_low, "SR_upper": sr_high,
            "CR_lower": cr_low, "CR_upper": cr_high,
            "TR_lower": tr_low, "TR_upper": tr_high,
            "DoorSR_lower": doorsr_low, "DoorSR_upper": doorsr_high,
            "PFR_lower": pfr_low, "PFR_upper": pfr_high,
        })

        formatted_row = {
            "Environment": environment,
            "Method": method,
            "N": int(len(group)),
            "SR (%)": f"{100.0 * numeric_row['SR']:.1f} [{100.0 * sr_low:.1f}, {100.0 * sr_high:.1f}]",
            "CR (%)": f"{100.0 * numeric_row['CR']:.1f} [{100.0 * cr_low:.1f}, {100.0 * cr_high:.1f}]",
            "TR (%)": f"{100.0 * numeric_row['TR']:.1f} [{100.0 * tr_low:.1f}, {100.0 * tr_high:.1f}]",
            "DoorSR (%)": f"{100.0 * numeric_row['DoorSR']:.1f} [{100.0 * doorsr_low:.1f}, {100.0 * doorsr_high:.1f}]",
            "PFR (%)": f"{100.0 * numeric_row['PFR']:.1f} [{100.0 * pfr_low:.1f}, {100.0 * pfr_high:.1f}]",
        }
        if condition is not None:
            formatted_row = {"Condition": condition, **formatted_row}

        for metric in SUCCESS_ONLY_METRICS:
            mean, std, ci95, n = mean_std_ci(successful[metric])
            numeric_row[f"{metric}_mean"] = mean
            numeric_row[f"{metric}_std"] = std
            numeric_row[f"{metric}_ci95"] = ci95
            numeric_row[f"{metric}_n_success"] = n
            formatted_row[metric] = fmt_mean_std_ci(successful[metric])

        mean, std, ci95, n = mean_std_ci(group["PlanningTime"])
        numeric_row["PlanningTime_mean"] = mean
        numeric_row["PlanningTime_std"] = std
        numeric_row["PlanningTime_ci95"] = ci95
        numeric_row["PlanningTime_n"] = n
        formatted_row["PlanningTime"] = fmt_mean_std_ci(group["PlanningTime"])

        numeric_rows.append(numeric_row)
        formatted_rows.append(formatted_row)

    numeric_df = pd.DataFrame(numeric_rows)
    formatted_df = pd.DataFrame(formatted_rows)

    numeric_path = "out/comparison/tables/performance_summary.csv"
    latex_table_path = "out/comparison/tables/performance_table.tex"
    latex_paper_path = "paper/overleaf_template/performance_table.tex"

    numeric_df.to_csv(numeric_path, index=False)
    with open(latex_table_path, "w", encoding="utf-8") as f:
        f.write(formatted_df.to_latex(index=False, escape=False))
    with open(latex_paper_path, "w", encoding="utf-8") as f:
        f.write(formatted_df.to_latex(index=False, escape=False))

    print(f"Numeric summary saved: {numeric_path}")
    print(f"LaTeX table saved: {latex_table_path} and {latex_paper_path}")


def run_statistical_analysis(df, environments):
    print("\n=== Step 4: Statistical analysis ===")
    if "ModelTag" in df.columns:
        df = df[df["ModelTag"].fillna("") == ""].copy()
    stat_path = "out/comparison/tables/statistical_tests.csv"

    if not _HAVE_SCIPY:
        pd.DataFrame().to_csv(stat_path, index=False)
        print("scipy is not installed; statistical tests were skipped.")
        return

    rows = []
    conditions = list(df["Condition"].dropna().unique()) if "Condition" in df.columns else [None]
    for condition in conditions:
        condition_df = df if condition is None else df[df["Condition"] == condition]
        for env_idx in environments:
            env_label = f"Ortam {env_idx}"
            env_df = condition_df[(condition_df["Environment"] == env_label) & (condition_df["Success"] == 1.0)]

            for metric in SUCCESS_ONLY_METRICS:
                groups = []
                for method in METHODS:
                    values = env_df[env_df["Method"] == method][metric].dropna().to_numpy(dtype=float)
                    if values.size >= 2:
                        groups.append((method, values))

                proposed = dict(groups).get(PROPOSED_METHOD)
                if len(groups) < 2 or proposed is None or proposed.size < 2:
                    continue

                combined = np.concatenate([values for _, values in groups])
                if np.unique(combined).size <= 1:
                    kw_stat, kw_p = 0.0, 1.0
                else:
                    kw_stat, kw_p = kruskal(*[values for _, values in groups])

                raw_p_values = []
                comp_records = []
                for baseline in BASELINES:
                    baseline_values = env_df[env_df["Method"] == baseline][metric].dropna().to_numpy(dtype=float)
                    if baseline_values.size < 2:
                        continue
                    _, raw_p = mannwhitneyu(proposed, baseline_values, alternative="two-sided")
                    raw_p_values.append(raw_p)
                    rec = {
                        "Environment": env_label,
                        "Metric": metric,
                        "Comparison": f"{PROPOSED_METHOD} vs {baseline}",
                        "KruskalWallis_H": float(kw_stat),
                        "KruskalWallis_p": float(kw_p),
                        "MWU_p_raw": float(raw_p),
                        "AstarDQN_n": int(proposed.size),
                        "Baseline_n": int(baseline_values.size),
                        "AstarDQN_median": float(np.median(proposed)),
                        "Baseline_median": float(np.median(baseline_values)),
                        "CliffsDelta": cliffs_delta(proposed, baseline_values),
                    }
                    if condition is not None:
                        rec = {"Condition": condition, **rec}
                    comp_records.append(rec)

                if raw_p_values:
                    reject, pvals_corrected = holm_bonferroni(raw_p_values, alpha=0.05)
                    for rec, corrected_p, is_rejected in zip(comp_records, pvals_corrected, reject):
                        rec["MWU_p_Holm"] = float(corrected_p)
                        rec["Significant_Holm_0.05"] = bool(is_rejected)
                        rows.append(rec)

    stat_df = pd.DataFrame(rows)
    stat_df.to_csv(stat_path, index=False)
    print(f"Statistical tests saved: {stat_path}")


def evaluate_method(env, env_idx, method_name, follower, seed, gif_seeds, condition_name="main", train_seed=None, eval_seed=None):
    res = follower.evaluate(seed=seed)
    if condition_name == "main" and len(res["xs"]) > 0 and eval_seed is not None and eval_seed <= gif_seeds:
        method_safe_name = method_name.replace("*", "").replace("+", "_")
        seed_tag = f"train{train_seed}_eval{eval_seed}"
        gif_path = f"out/ortam{env_idx}/animations/{method_safe_name}_{seed_tag}.gif"
        traj_img_path = f"paper/figures/{method_safe_name}_ortam{env_idx}_{seed_tag}.png"
        try:
            create_animation_gif(env, res, method_name=f"{method_name} ({seed_tag})", save_path=gif_path)
            save_static_trajectory(env, res, method_name=f"{method_name}", save_path=traj_img_path)
        except Exception as exc:
            print(f"  Visualization error for {method_name}, {seed_tag}: {exc}")
    return res


def main():
    args = build_arg_parser().parse_args()
    environments = parse_envs(args.envs)
    train_episodes = parse_train_episodes(args.train_episodes)
    train_seeds = parse_int_list(args.train_seeds)
    if args.eval_model_seed is not None:
        eval_model_seeds = [args.eval_model_seed]
    elif args.eval_model_seeds:
        eval_model_seeds = parse_int_list(args.eval_model_seeds)
    else:
        eval_model_seeds = train_seeds
    conditions = build_experiment_conditions(args)
    ensure_output_dirs(environments)

    print("Q1 SCI robotics comparison pipeline started.")
    print(f"Environments: {environments}")
    print(f"Training seeds: {train_seeds}")
    print(f"Evaluation model seeds: {eval_model_seeds}")
    print(f"Evaluation seeds per method/environment/train seed: {args.num_seeds}")
    print(f"Evaluation seed range for this run: {args.eval_seed_start}-{args.num_seeds}")
    print(f"Experiment conditions: {[cond['name'] for cond in conditions]}")
    if args.eval_seed_start < 1:
        raise ValueError("--eval-seed-start must be >= 1")
    if args.eval_seed_start > args.num_seeds:
        raise ValueError("--eval-seed-start cannot be larger than --num-seeds")
    if args.checkpoint_every < 1:
        raise ValueError("--checkpoint-every must be >= 1")

    if not args.skip_training:
        print("\n=== Step 1: DQN training ===")
        for env_idx in environments:
            eps = train_episodes.get(env_idx, 1000)
            for train_seed in train_seeds:
                model_path_astar = dqn_model_path(env_idx, train_seed)
                if not os.path.exists(model_path_astar):
                    train_dqn(env_idx=env_idx, num_episodes=eps,
                              output_dir=f"out/ortam{env_idx}/training/", use_astar=True, seed=train_seed)
                else:
                    print(f"Environment {env_idx}, seed {train_seed}: A*+DQN model found; training skipped.")

                for condition in conditions:
                    model_tag = model_tag_for_condition(condition)
                    if model_tag is None:
                        continue
                    model_path_ablation = dqn_model_path(env_idx, train_seed, model_tag=model_tag)
                    if not os.path.exists(model_path_ablation):
                        train_dqn(env_idx=env_idx, num_episodes=eps,
                                  output_dir=f"out/ortam{env_idx}/training/",
                                  use_astar=True, seed=train_seed,
                                  env_kwargs=env_kwargs_from_condition(condition, args, include_eval_params=False),
                                  model_tag=model_tag)
                    else:
                        print(f"Environment {env_idx}, seed {train_seed}: {model_tag} model found; training skipped.")
    else:
        print("\n=== Step 1: DQN training skipped by user ===")

    print("\n=== Step 2: Evaluation ===")
    metrics_path = args.metrics_path
    checkpoint_path = checkpoint_path_for(metrics_path)

    if args.resume:
        resume_df = load_resume_rows(metrics_path, checkpoint_path)
        results_list = resume_df.to_dict("records") if not resume_df.empty else []
        completed_keys = {result_key_from_row(row) for row in results_list}
        print(
            f"Resume mode: loaded {len(results_list)} completed method-runs "
            f"from {metrics_path} / {checkpoint_path}."
        )
        print("Resume assumes the experiment settings are unchanged.")
    else:
        results_list = []
        completed_keys = set()
        if os.path.exists(checkpoint_path):
            os.remove(checkpoint_path)

    new_rows_since_checkpoint = 0

    def store_result(row):
        nonlocal new_rows_since_checkpoint
        key = result_key_from_row(row)
        if key in completed_keys:
            return False
        results_list.append(row)
        completed_keys.add(key)
        new_rows_since_checkpoint += 1
        if new_rows_since_checkpoint >= args.checkpoint_every:
            save_evaluation_checkpoint(results_list, checkpoint_path)
            new_rows_since_checkpoint = 0
        return True

    condition_index = {cond["name"]: idx for idx, cond in enumerate(conditions)}
    for condition in conditions:
        cond_name = condition["name"]
        print(f"\nEvaluating condition: {cond_name}")

        # Seed-major evaluation order: completely finish seed 42, then 100, then 2024.
        for train_seed in eval_model_seeds:
            print(f"\n  === Train seed {train_seed} ===", flush=True)

            for env_idx in environments:
                print(f"    Environment {env_idx}...", flush=True)
                model_tag = model_tag_for_condition(condition)
                dqn_astar_path = dqn_model_path(env_idx, train_seed, model_tag=model_tag)

                if PROPOSED_METHOD in condition["methods"] and not os.path.exists(dqn_astar_path):
                    raise FileNotFoundError(
                        f"Required model not found: {dqn_astar_path}. "
                        "Evaluation stopped instead of using random weights."
                    )

                for eval_seed in range(args.eval_seed_start, args.num_seeds + 1):
                    # Skip an evaluation seed completely when all requested methods are already present.
                    missing_methods = [
                        method_name for method_name in condition["methods"]
                        if expected_result_key(condition, env_idx, method_name, train_seed, eval_seed) not in completed_keys
                    ]
                    if not missing_methods:
                        continue

                    if eval_seed == args.eval_seed_start or eval_seed % 10 == 0 or eval_seed == args.num_seeds:
                        print(
                            f"      Train seed {train_seed}, eval seed {eval_seed}/{args.num_seeds} "
                            f"(remaining methods: {len(missing_methods)})",
                            flush=True,
                        )

                    trial_seed = make_trial_seed(train_seed, eval_seed, condition_index[cond_name])
                    try:
                        env = MobileRobotEnv(
                            env_idx=env_idx,
                            is_eval=True,
                            **env_kwargs_from_condition(condition, args, include_eval_params=True),
                        )
                    except RuntimeError as exc:
                        for method_name in missing_methods:
                            store_result({
                                "Condition": cond_name,
                                "Environment": f"Ortam {env_idx}",
                                "Method": method_name,
                                "TrainSeed": train_seed,
                                "EvalSeed": eval_seed,
                                "TrialSeed": trial_seed,
                                "Outcome": "planning_failure",
                                "Success": 0.0,
                                "Collision": 0.0,
                                "Timeout": 0.0,
                                "PlanningFailure": 1.0,
                                "DoorSuccess": 0.0,
                                "PathLength": 0.0,
                                "Time": 0.0,
                                "Smoothness": 0.0,
                                "ControlEnergy": 0.0,
                                "MinDist": 0.0,
                                "MeanDist": 0.0,
                                "PlanningTime": 0.0,
                                "ControlTimeMean": 0.0,
                                "ControlTimeP95": 0.0,
                                "LidarNoise": condition["lidar_noise"],
                                "DynObsSpeed": condition["dyn_obs_speed"],
                                "StaticDensity": condition["static_density"],
                                "UseLookAhead": condition["use_look_ahead"],
                                "UseDoorMask": condition["use_door_mask"],
                                "UseLidar": condition["use_lidar"],
                                "NoDenseReward": condition["ablation_no_dense_reward"],
                                "Error": str(exc),
                            })
                        continue

                    followers = {
                        "A*+DWA": StandardDWAFollower(env, use_rrt=False),
                        "A*+Predictive-DWA": PredictiveDWAFollower(env, use_rrt=False),
                        PROPOSED_METHOD: DQNFollower(env, model_path=dqn_astar_path, use_astar=True),
                    }

                    for method_name in missing_methods:
                        follower = followers[method_name]
                        res = evaluate_method(
                            env,
                            env_idx,
                            method_name,
                            follower,
                            trial_seed,
                            args.gif_seeds,
                            condition_name=cond_name,
                            train_seed=train_seed,
                            eval_seed=eval_seed,
                        )

                        store_result({
                            "Condition": cond_name,
                            "Environment": f"Ortam {env_idx}",
                            "Method": method_name,
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
                            "LidarNoise": condition["lidar_noise"],
                            "DynObsSpeed": condition["dyn_obs_speed"],
                            "StaticDensity": condition["static_density"],
                            "UseLookAhead": condition["use_look_ahead"],
                            "UseDoorMask": condition["use_door_mask"],
                            "UseLidar": condition["use_lidar"],
                            "NoDenseReward": condition["ablation_no_dense_reward"],
                            "Error": "",
                        })

                # Force a checkpoint at each environment boundary, even if checkpoint_every is larger.
                save_evaluation_checkpoint(results_list, checkpoint_path)
                new_rows_since_checkpoint = 0
                print(
                    f"    Environment {env_idx} complete for train seed {train_seed}. "
                    f"Checkpoint: {checkpoint_path}",
                    flush=True,
                )

            print(f"  === Train seed {train_seed} complete ===", flush=True)

    # Persist any rows completed since the last checkpoint before final aggregation.
    save_evaluation_checkpoint(results_list, checkpoint_path)

    df = dedupe_and_sort_results(pd.DataFrame(results_list))
    if args.append_results and os.path.exists(metrics_path):
        previous_df = pd.read_csv(metrics_path)
        df = dedupe_and_sort_results(pd.concat([previous_df, df], ignore_index=True, sort=False))

    atomic_write_csv(df, metrics_path)
    print(f"\nAll run-level metrics saved: {metrics_path}")

    create_plots(df)
    run_statistical_analysis(df, environments)
    create_performance_tables(df)

    if os.path.exists(checkpoint_path):
        os.remove(checkpoint_path)
        print(f"Evaluation checkpoint cleared: {checkpoint_path}")

    print("\n=== Pipeline completed ===")


if __name__ == "__main__":
    main()
