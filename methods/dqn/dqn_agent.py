import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque

class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(QNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim)
        )
        
    def forward(self, x):
        return self.fc(x)


class DuelingQNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DuelingQNetwork, self).__init__()
        self.feature = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.LayerNorm(256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.LayerNorm(256),
            nn.ReLU(),
        )
        self.value = nn.Sequential(
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
        )
        self.advantage = nn.Sequential(
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
        )

    def forward(self, x):
        z = self.feature(x)
        value = self.value(z)
        advantage = self.advantage(z)
        return value + advantage - advantage.mean(dim=1, keepdim=True)

class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)
        
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
        
    def sample(self, batch_size):
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return (np.array(state, dtype=np.float32),
                np.array(action, dtype=np.int64),
                np.array(reward, dtype=np.float32),
                np.array(next_state, dtype=np.float32),
                np.array(done, dtype=np.float32))
                
    def __len__(self):
        return len(self.buffer)


class PrioritizedReplayBuffer:
    def __init__(self, capacity, alpha=0.6, eps=1e-5):
        self.capacity = int(capacity)
        self.alpha = alpha
        self.eps = eps
        self.buffer = []
        self.priorities = np.zeros((self.capacity,), dtype=np.float32)
        self.pos = 0
        self.max_priority = 1.0

    def push(self, state, action, reward, next_state, done):
        item = (state, action, reward, next_state, done)
        if len(self.buffer) < self.capacity:
            self.buffer.append(item)
        else:
            self.buffer[self.pos] = item
        self.priorities[self.pos] = self.max_priority
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size, beta=0.4):
        priorities = self.priorities[:len(self.buffer)]
        probs = priorities ** self.alpha
        probs /= probs.sum()
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)
        samples = [self.buffer[idx] for idx in indices]
        state, action, reward, next_state, done = zip(*samples)

        weights = (len(self.buffer) * probs[indices]) ** (-beta)
        weights /= weights.max()
        return (
            np.array(state, dtype=np.float32),
            np.array(action, dtype=np.int64),
            np.array(reward, dtype=np.float32),
            np.array(next_state, dtype=np.float32),
            np.array(done, dtype=np.float32),
            indices,
            np.array(weights, dtype=np.float32),
        )

    def update_priorities(self, indices, priorities):
        for idx, priority in zip(indices, priorities):
            priority = float(abs(priority) + self.eps)
            self.priorities[idx] = priority
            self.max_priority = max(self.max_priority, priority)

    def __len__(self):
        return len(self.buffer)

from config import Config

class DQNAgent:
    def __init__(self, state_dim=24, action_dim=7, lr=Config.LR, gamma=Config.GAMMA,
                 buffer_capacity=Config.BUFFER_CAPACITY, batch_size=Config.BATCH_SIZE, tau=Config.TAU,
                 dueling=False, prioritized_replay=False, n_step=1, reward_clip=None):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.batch_size = batch_size
        self.tau = tau
        self.dueling = dueling
        self.prioritized_replay = prioritized_replay
        self.n_step = max(1, int(n_step))
        self.reward_clip = reward_clip
        self.learn_steps = 0
        self.n_step_buffer = deque(maxlen=self.n_step)
        
        # Cihaz ayarı (CUDA varsa GPU kullan)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Ağırlık Kümesi
        network_cls = DuelingQNetwork if self.dueling else QNetwork
        self.q_local = network_cls(state_dim, action_dim).to(self.device)
        self.q_target = network_cls(state_dim, action_dim).to(self.device)
        self.q_target.load_state_dict(self.q_local.state_dict())
        
        self.optimizer = optim.Adam(self.q_local.parameters(), lr=lr)
        if self.prioritized_replay:
            self.memory = PrioritizedReplayBuffer(buffer_capacity, alpha=Config.PER_ALPHA)
        else:
            self.memory = ReplayBuffer(buffer_capacity)
        
        # Açısal Hız Aksiyonları (DQN çıkış indekslerini eşle)
        self.actions_w = np.array([-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2], dtype=np.float32)

    def act(self, state, epsilon=0.0):
        if np.random.rand() < epsilon:
            return np.random.randint(self.action_dim)
            
        state_t = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        self.q_local.eval()
        with torch.no_grad():
            action_values = self.q_local(state_t)
        self.q_local.train()
        
        return int(torch.argmax(action_values).item())

    def get_w_action(self, action_idx):
        return float(self.actions_w[action_idx])

    def _make_n_step_transition(self):
        state, action, _, _, _ = self.n_step_buffer[0]
        reward = 0.0
        next_state = self.n_step_buffer[-1][3]
        done = self.n_step_buffer[-1][4]
        for idx, (_, _, r, ns, d) in enumerate(self.n_step_buffer):
            reward += (self.gamma ** idx) * r
            next_state = ns
            done = d
            if d:
                break
        return state, action, reward, next_state, done

    def step(self, state, action, reward, next_state, done):
        if self.reward_clip is not None:
            reward = float(np.clip(reward, -self.reward_clip, self.reward_clip))
        if self.n_step <= 1:
            self.memory.push(state, action, reward, next_state, done)
            return

        self.n_step_buffer.append((state, action, reward, next_state, done))
        if len(self.n_step_buffer) == self.n_step:
            self.memory.push(*self._make_n_step_transition())

        if done:
            while len(self.n_step_buffer) > 1:
                self.n_step_buffer.popleft()
                self.memory.push(*self._make_n_step_transition())
            self.n_step_buffer.clear()

    def learn(self):
        if len(self.memory) < self.batch_size:
            return

        beta = min(1.0, Config.PER_BETA_START + self.learn_steps * (1.0 - Config.PER_BETA_START) / Config.PER_BETA_FRAMES)
        if self.prioritized_replay:
            states, actions, rewards, next_states, dones, indices, weights = self.memory.sample(self.batch_size, beta=beta)
        else:
            states, actions, rewards, next_states, dones = self.memory.sample(self.batch_size)
            indices = None
            weights = np.ones_like(rewards, dtype=np.float32)
        
        states_t = torch.FloatTensor(states).to(self.device)
        actions_t = torch.LongTensor(actions).unsqueeze(1).to(self.device)
        rewards_t = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states_t = torch.FloatTensor(next_states).to(self.device)
        dones_t = torch.FloatTensor(dones).unsqueeze(1).to(self.device)
        weights_t = torch.FloatTensor(weights).unsqueeze(1).to(self.device)
        
        # Double DQN Güncelleme Mekanizması
        # Local ağdan en iyi aksiyonu seç: a* = argmax_a Q_local(s', a)
        self.q_local.eval()
        with torch.no_grad():
            next_local_actions = torch.argmax(self.q_local(next_states_t), dim=1, keepdim=True)
        self.q_local.train()
        
        # Target ağdan bu aksiyonun değerini oku: Q_target(s', a*)
        with torch.no_grad():
            next_target_q = self.q_target(next_states_t).gather(1, next_local_actions)
            
        target_q = rewards_t + ((self.gamma ** self.n_step) * next_target_q * (1 - dones_t))
        
        current_q = self.q_local(states_t).gather(1, actions_t)
        
        td_error = target_q - current_q
        loss = (nn.SmoothL1Loss(reduction="none")(current_q, target_q) * weights_t).mean()
        
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.q_local.parameters(), Config.GRAD_CLIP_NORM)
        self.optimizer.step()
        if self.prioritized_replay and indices is not None:
            self.memory.update_priorities(indices, td_error.detach().abs().cpu().numpy().flatten())
        
        # Target Network Soft-Update
        self.soft_update()
        self.learn_steps += 1

    def soft_update(self):
        for target_param, local_param in zip(self.q_target.parameters(), self.q_local.parameters()):
            target_param.data.copy_(self.tau * local_param.data + (1.0 - self.tau) * target_param.data)
            
    def save(self, filepath):
        torch.save({
            "model_state": self.q_local.state_dict(),
            "dueling": self.dueling,
            "prioritized_replay": self.prioritized_replay,
            "n_step": self.n_step,
            "reward_clip": self.reward_clip,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
        }, filepath)
        
    def load(self, filepath):
        checkpoint = torch.load(filepath, map_location=self.device)
        if isinstance(checkpoint, dict) and "model_state" in checkpoint:
            needs_rebuild = bool(checkpoint.get("dueling", False)) != self.dueling
            if needs_rebuild:
                self.dueling = bool(checkpoint.get("dueling", False))
                network_cls = DuelingQNetwork if self.dueling else QNetwork
                self.q_local = network_cls(self.state_dim, self.action_dim).to(self.device)
                self.q_target = network_cls(self.state_dim, self.action_dim).to(self.device)
                self.optimizer = optim.Adam(self.q_local.parameters(), lr=Config.LR)
            checkpoint = checkpoint["model_state"]
        self.q_local.load_state_dict(checkpoint)
        self.q_target.load_state_dict(self.q_local.state_dict())
