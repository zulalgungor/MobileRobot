# MobileRobot

## Task-Aware Astar with DDQN-Guided Predictive Shielding for Sequential-Task Navigation in Dynamic Environments

This repository contains the simulation, training, evaluation, and result files developed for a mobile robot navigation study in dynamic environments with sequential task constraints.

The proposed framework combines **Task-Aware Astar** global planning, a **Double Deep Q-Network (DDQN)** based local decision policy, and a **predictive safety shield** for dynamic obstacle avoidance.

The system is evaluated in three simulated environments containing static obstacles, dynamic obstacles, buttons, and controlled doors.

---

## Proposed Method

The proposed method is:

**Task-Aware Astar with DDQN-Guided Predictive Shielding**

The navigation architecture combines three main components:

1. **Task-Aware Astar Global Planning**  
   The global planner represents task progress within the search state. Button activation and door availability are considered during route generation so that the planned path respects the required sequential tasks.

2. **DDQN-Based Local Decision Making**  
   A DDQN agent provides local control guidance using robot state, path information, obstacle information, and LiDAR observations.

3. **Predictive Safety Shielding**  
   Candidate control commands are checked using short-horizon motion prediction. The safety layer considers static obstacles, doors, and predicted dynamic-obstacle motion before selecting an executable command.

This structure allows learned local navigation decisions to operate together with global task planning and explicit safety checks.

---

## Compared Methods

Three navigation approaches are evaluated:

| Method | Description |
|---|---|
| **Astar + DWAstar* | Astar global planning with standard Dynamic Window Approach local control |
| **Astar + Predictive-DWAstar* | Astar global planning with predictive dynamic-obstacle handling |
| **Task-Aware Astar + DDQN-Guided Predictive Shielding** | Proposed hybrid learning and safety architecture |

---

## Experimental Environments

The framework contains three simulated navigation environments.

### Environment 1

The first environment provides a comparatively simpler sequential navigation task while still requiring interaction with the button-door mechanism and obstacle avoidance.

### Environment 2

The second environment introduces a longer navigation route and increased interaction with dynamic obstacles and constrained passages.

### Environment 3

The third environment provides the most complex navigation layout among the three evaluated scenarios, combining multiple constrained regions, dynamic obstacles, and sequential task requirements.

---

## Experimental Protocol

Three independent training seeds are used:

```text
42
100
2024
```

For each trained model, **200 evaluation scenarios** are used.

The main comparison therefore contains:

```text
3 environments
× 3 navigation methods
× 3 training seeds
× 200 evaluation scenarios
= 5400 evaluation runs
```

The evaluation includes randomized starting conditions and dynamic-obstacle phase variation.

The main configuration is defined in:

```text
config.py
```

---

## Main Configuration

Important training parameters include:

| Parameter | Value |
|---|---:|
| Training seeds | 42, 100, 2024 |
| Evaluation seeds per training seed | 200 |
| Learning rate | 3e-4 |
| Discount factor | 0.99 |
| Replay buffer capacity | 100000 |
| Batch size | 64 |
| Soft target update coefficient | 0.005 |
| Prioritized replay alpha | 0.6 |
| Prioritized replay beta start | 0.4 |
| N-step return | 3 |
| Reward clipping | 10.0 |
| Gradient clipping norm | 10.0 |
| Initial epsilon | 1.0 |
| Minimum epsilon | 0.05 |
| Epsilon decay | 0.99 |

The enhanced DDQN configuration uses prioritized experience replay, multi-step returns, and a dueling network architecture.

---

## Repository Structure

```text
MobileRobot/
│
├── env/
│   └── mobile_robot_env.py
│
├── methods/
│   ├── followers.py
│   ├── visualizer.py
│   └── dqn/
│       ├── dqn_agent.py
│       └── dqn_trainer.py
│
├── training/
│   ├── train_env1.py
│   ├── train_env2.py
│   └── train_env3.py
│
├── evaluation/
│   ├── test_env1.py
│   ├── test_env2.py
│   └── test_env3.py
│
├── results/
│   ├── metrics.csv
│   └── figures/
│       ├── figure_1.svg
│       ├── figure_2.svg
│       ├── figure_3.svg
│       ├── figure_4.svg
│       └── figure_5.svg
│
├── config.py
├── run_all.py
├── eval_checkpoint.py
├── requirements.txt
├── .gitignore
└── README.md
```

---

## File Description

### `env/mobile_robot_env.py`

Contains the mobile robot simulation environment, including:

- robot kinematics
- environment maps
- static obstacles
- dynamic obstacles
- LiDAR simulation
- button and door mechanisms
- collision checking
- task-aware global path planning
- reward calculation
- evaluation metrics

### `methods/followers.py`

Contains the navigation controllers used in the comparison:

- Standard DWA follower
- Predictive DWA follower
- DDQN-guided follower
- predictive safety checking
- candidate control evaluation
- dynamic-obstacle prediction

### `methods/dqn/dqn_agent.py`

Contains the neural-network and reinforcement-learning components, including the DDQN agent, replay buffers, and enhanced network implementation.

### `methods/dqn/dqn_trainer.py`

Contains the DDQN training procedure.

### `methods/visualizer.py`

Contains trajectory and environment visualization utilities.

### `training/`

Contains environment-specific training scripts.

### `evaluation/`

Contains auxiliary environment-specific evaluation scripts.

For the full multi-seed experimental evaluation, `run_all.py` and `eval_checkpoint.py` are the main evaluation utilities.

### `results/metrics.csv`

Contains the run-level evaluation results used for the experimental analysis.

### `results/figures/`

Contains the final figures used in the manuscript.

---

## Installation

Clone the repository:

```bash
git clone https://github.com/zulalgungor/MobileRobot.git
cd MobileRobot
```

Create a virtual environment if desired:

```bash
python -m venv .venv
```

On Windows:

```bash
.venv\Scripts\activate
```

On Linux/macOS:

```bash
source .venv/bin/activate
```

Install the required dependencies:

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
numpy
pandas
matplotlib
seaborn
scipy
torch
Pillow
```

---

## Training

Environment-specific training can be started using the training modules.

### Environment 1

```bash
python -m training.train_env1
```

### Environment 2

```bash
python -m training.train_env2
```

### Environment 3

```bash
python -m training.train_env3
```

The configured training seeds are:

```text
42, 100, 2024
```

The environment-specific training budgets are:

```text
Environment 1: 1000 episodes
Environment 2: 1500 episodes
Environment 3: 1500 episodes
```

Generated model checkpoints and training outputs are stored under the local `out/` directory.

Model checkpoint files are not tracked in this repository by default.

---

## Running the Main Experiment Pipeline

The complete experiment pipeline can be started with:

```bash
python run_all.py
```

The main runner supports command-line options for:

- selecting environments
- selecting training seeds
- selecting evaluation model seeds
- changing the number of evaluation seeds
- skipping training when checkpoints already exist
- robustness experiments
- selected ablation conditions
- evaluation start perturbations
- LiDAR noise
- dynamic-obstacle speed
- static-obstacle density

To evaluate existing trained models without retraining:

```bash
python run_all.py --skip-training
```

This command requires the corresponding trained model checkpoint files to already exist under the expected `out/` directories.

---

## Checkpointed Evaluation

For long evaluation runs, the repository also contains:

```text
eval_checkpoint.py
```

This script saves evaluation results incrementally.

Example for training seed 42:

```bash
python eval_checkpoint.py --envs 1,2,3 --train-seed 42 --eval-seed-start 1 --eval-seed-end 200
```

Training seed 100:

```bash
python eval_checkpoint.py --envs 1,2,3 --train-seed 100 --eval-seed-start 1 --eval-seed-end 200
```

Training seed 2024:

```bash
python eval_checkpoint.py --envs 1,2,3 --train-seed 2024 --eval-seed-start 1 --eval-seed-end 200
```

Evaluation results are written to:

```text
out/comparison/metrics.csv
```

---

## Evaluation Metrics

The evaluation records both task success and continuous navigation performance.

| Metric | Description |
|---|---|
| Success | Successful completion of the navigation task |
| Collision | Collision occurrence |
| Timeout | Failure caused by reaching the maximum evaluation duration |
| PlanningFailure | Global planning failure |
| DoorSuccess | Successful completion of the door-related task |
| PathLength | Travelled path length |
| Time | Task completion time |
| Smoothness | Control smoothness metric |
| ControlEnergy | Control effort metric |
| MinDist | Minimum obstacle clearance |
| MeanDist | Mean obstacle clearance |
| PlanningTime | Global planning computation time |
| ControlTimeMean | Mean local-control computation time |
| ControlTimeP95 | 95th percentile local-control computation time |

Continuous navigation metrics are analyzed on successful trials.

---

## Main Results

The following success rates were obtained in the main evaluation:

| Environment | Astar + DWA | Astar + Predictive-DWA | Proposed |
|---|---:|---:|---:|
| Environment 1 | 79.0% | 94.3% | **99.0%** |
| Environment 2 | 15.8% | 48.0% | **87.2%** |
| Environment 3 | 29.8% | 60.5% | **92.8%** |

Collision rates were:

| Environment | Astar + DWA | Astar + Predictive-DWA | Proposed |
|---|---:|---:|---:|
| Environment 1 | 21.0% | 5.7% | **1.0%** |
| Environment 2 | 84.2% | 52.0% | **12.8%** |
| Environment 3 | 70.2% | 39.5% | **7.2%** |

The proposed architecture achieved the highest success rate and the lowest collision rate in all three evaluated environments.

Run-level data supporting the reported results are available in:

```text
results/metrics.csv
```

---

## Statistical Analysis

The experimental analysis includes comparisons of success outcomes and continuous navigation metrics.

Continuous metrics are evaluated on successful trials using non-parametric statistical tests and effect-size analysis.

The analysis includes:

```text
Kruskal-Wallis test
Mann-Whitney U test
Holm correction
Cliff's delta
```

Cliff's delta is computed as:

```text
δ = (number of greater pairs - number of lower pairs) / (nx × ny)
```

---

## Result Files

The main evaluation dataset is available at:

```text
results/metrics.csv
```

This file contains run-level results for the evaluated environments, navigation methods, training seeds, and evaluation scenarios.

Figures used in the manuscript are stored in:

```text
results/figures/
```

Some final manuscript visualizations were prepared from the exported evaluation results using spreadsheet software. The underlying numerical evaluation data are provided in `metrics.csv`.

Generated intermediate files, temporary plots, trained model checkpoints, and other runtime outputs are excluded from version control through `.gitignore`.

---

## Reproducibility Notes

The study uses three independent training seeds and multiple evaluation scenarios to reduce dependence on a single trained model or a single simulation run.

Evaluation conditions include variation in:

- initial robot state
- initial orientation
- dynamic-obstacle phase
- trained model seed

The main evaluation dataset contains **5400 run-level observations**.

Because trained neural-network checkpoint files can be large, they are excluded from the Git repository. Models can be regenerated using the provided training scripts and configuration.

---

## Robustness and Ablation Support

The experiment runner contains options for additional robustness and component-level investigations, including:

```text
LiDAR noise
dynamic-obstacle speed
static-obstacle density
look-ahead configuration
LiDAR availability
dense reward configuration
door-mask configuration
```

These options are provided by the experimental framework for controlled investigations.

The main manuscript results reported in this repository correspond to the principal three-method comparison. Additional experimental switches in the code should not be interpreted as reported manuscript experiments unless corresponding result data are explicitly provided.

---

## Associated Study

This repository accompanies the unpublished manuscript:

**“Task-Aware Astar with DDQN-Guided Predictive Shielding for Sequential-Task Navigation in Dynamic Environments”**

Turkish title:

**“Dinamik Ortamlarda Sıralı Görev Navigasyonu için Görev Farkındalıklı Astar ile DDQN Rehberli Öngörülü Güvenlik Kalkanı”**

The manuscript has not yet been published.

---

## Citation

Citation information will be added after publication.

---

## Contact

For questions related to the implementation, experiments, or result files, please use the GitHub repository issue tracker.

---

## Acknowledgment

This repository is intended to support transparency and reproducibility of the simulation-based experiments associated with the manuscript.
