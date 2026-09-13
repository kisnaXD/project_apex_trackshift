# f1sim/planning/live_rl_agent.py
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import numpy as np
import os

class ActorCritic(nn.Module):
    def __init__(self, state_dim=8, action_dim=2):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh()
        )
        self.actor_mean = nn.Linear(64, action_dim)
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))
        self.critic = nn.Linear(64, 1)

    def forward(self, state):
        feat = self.shared(state)
        mean = torch.tanh(self.actor_mean(feat))
        std = torch.exp(self.actor_logstd)
        value = self.critic(feat)
        return Normal(mean, std), value

    def act_deterministic(self, state_np):
        with torch.no_grad():
            state = torch.as_tensor(state_np, dtype=torch.float32).unsqueeze(0)
            feat = self.shared(state)
            mean = torch.tanh(self.actor_mean(feat))
            return mean.squeeze(0).numpy()

def train_live_agent(total_timesteps=10000, checkpoint_path="assets/ppo_f1_2026.pt"):
    from f1sim.planning.rl_env import F1TrackShiftEnv
    env = F1TrackShiftEnv()
    model = ActorCritic()
    optimizer = optim.Adam(model.parameters(), lr=3e-4)

    obs, _ = env.reset()
    for step in range(total_timesteps):
        state_t = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
        dist, value = model(state_t)
        action = dist.sample()
        action_np = action.squeeze(0).detach().numpy()

        next_obs, reward, done, _, _ = env.step(action_np)

        # Baseline loss minimization
        _, next_value = model(torch.as_tensor(next_obs, dtype=torch.float32).unsqueeze(0))
        target = reward + (0.99 * next_value * (1.0 - float(done)))
        critic_loss = nn.functional.mse_loss(value, target.detach())
        actor_loss = -dist.log_prob(action).mean() * (target - value).detach()

        loss = actor_loss + 0.5 * critic_loss
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        obs = next_obs if not done else env.reset()[0]

    os.makedirs(os.path.dirname(checkpoint_path), exist_ok=True)
    torch.save(model.state_dict(), checkpoint_path)
    print(f"Model saved to {checkpoint_path}")

if __name__ == "__main__":
    train_live_agent()