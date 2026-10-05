from env.mobile_robot_env import MobileRobotEnv
from methods.followers import DQNFollower

env = MobileRobotEnv(env_idx=3, is_eval=True)

astar_model = "out/ortam3/training/dqn_astar_ortam3.pth"
dqn_model = "out/ortam3/training/dqn_only_ortam3.pth"

f_astar = DQNFollower(env, model_path=astar_model, use_astar=True)
f_only = DQNFollower(env, model_path=dqn_model, use_astar=False)

succ_astar = 0
succ_only = 0
tests = 50

for i in range(tests):
    env.use_astar = True
    res = f_astar.evaluate(seed=i)
    if res["Success"]: succ_astar += 1
    
    env.use_astar = False
    res = f_only.evaluate(seed=i)
    if res["Success"]: succ_only += 1

print(f"A*+DQN Success (Env 3): {succ_astar}/{tests} ({(succ_astar/tests)*100}%)")
print(f"DQN-only Success (Env 3): {succ_only}/{tests} ({(succ_only/tests)*100}%)")
