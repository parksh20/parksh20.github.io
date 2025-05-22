# Isaac Lab 고급 토픽: 커스텀 RL 에이전트/정책 개발

Isaac Lab은 skrl (skrl.org)과 같은 기존 강화학습(RL) 라이브러리와의 긴밀한 통합을 통해 다양한 표준 RL 알고리즘을 쉽게 사용할 수 있도록 지원합니다. 하지만 특정 연구 목적이나 복잡한 문제 해결을 위해 기본 제공되는 알고리즘 외에 자신만의 커스텀 RL 알고리즘을 통합하거나, 에이전트의 신경망 아키텍처를 세밀하게 조정해야 할 필요가 있습니다.

이 섹션에서는 Isaac Lab 환경에서 커스텀 RL 알고리즘을 통합하는 방법과 정책(policy) 및 가치 함수(value function)를 위한 신경망 아키텍처를 사용자의 필요에 맞게 커스터마이징하는 전략에 대해 논의합니다.

## 1. Skrl 라이브러리 외의 커스텀 알고리즘 통합

Isaac Lab은 특정 RL 라이브러리에 종속되지 않고, Gymnasium (구 OpenAI Gym) 인터페이스와 유사한 표준화된 RL 환경 인터페이스(`ManagerBasedRLEnv` 등)를 제공합니다. 이는 skrl 외에도 Stable Baselines3, RSL-RL, 또는 사용자가 직접 개발한 RL 알고리즘 라이브러리와도 Isaac Lab 환경을 연동할 수 있는 유연성을 제공한다는 의미입니다.

**통합 전략:**

1.  **Gymnasium 호환성 확인:**
    *   사용하려는 RL 라이브러리 또는 직접 구현한 알고리즘이 Gymnasium의 `Env` 인터페이스를 따르는 환경과 상호작용할 수 있는지 확인합니다.
    *   Isaac Lab의 `ManagerBasedRLEnv`는 Gymnasium `Env`를 상속받으므로, 관찰 공간(observation space), 액션 공간(action space), `step()`, `reset()` 등의 메서드 시그니처가 호환되어야 합니다.
2.  **데이터 형식 변환:**
    *   Isaac Lab 환경은 주로 PyTorch 텐서(Tensor) 형태로 관찰 값을 반환하고 액션을 입력받습니다.
    *   사용하려는 RL 라이브러리가 다른 데이터 형식(예: NumPy 배열)을 사용한다면, 환경과 에이전트 사이에서 적절한 데이터 형식 변환 계층(wrapper 또는 utility function)을 추가해야 합니다.
    *   예를 들어, RL 에이전트가 NumPy 배열을 입력으로 받는다면, Isaac Lab 환경에서 받은 PyTorch 텐서를 `.cpu().numpy()`를 통해 변환하여 전달해야 합니다. 반대로, 에이전트가 출력한 NumPy 액션은 `torch.tensor()`를 사용하여 PyTorch 텐서로 변환 후 환경에 전달합니다.
3.  **학습 루프(Training Loop) 관리:**
    *   Skrl과 같은 라이브러리는 자체적인 학습 루프 관리 기능을 제공하지만, 커스텀 알고리즘을 사용하는 경우 학습 루프를 직접 구성해야 할 수 있습니다.
    *   학습 루프는 일반적으로 다음 단계를 포함합니다:
        1.  환경 리셋 (`env.reset()`)
        2.  에이전트로부터 액션 계산 (`agent.compute_action(observation)`)
        3.  환경 스텝 진행 (`env.step(action)`) 및 다음 관찰, 보상, 종료 여부 수신
        4.  경험 데이터 저장 (Replay Buffer 또는 Trajectory Buffer)
        5.  일정 조건 만족 시 에이전트 업데이트 (`agent.update()`)
        6.  에피소드 종료 또는 최대 스텝 도달 시 1단계로 복귀
    *   Isaac Lab 환경은 병렬 환경 실행(vectorized environments)을 지원하므로, 알고리즘과 학습 루프도 이를 고려하여 설계하는 것이 좋습니다.
4.  **Isaac Lab의 유틸리티 활용:**
    *   Isaac Lab (Orbit)은 환경 설정 로드, 시뮬레이션 애플리케이션 관리, 로깅 등 RL 학습 외적인 부분에서 유용한 유틸리티들을 제공합니다. 커스텀 알고리즘을 통합하더라도 이러한 유틸리티들은 계속 활용할 수 있습니다.

**예시: 간단한 커스텀 REINFORCE 알고리즘 통합 (개념 코드)**

```python
# --- 필요한 라이브러리 임포트 ---
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical # 이산적 액션 공간용 예시
# from torch.distributions import Normal # 연속적 액션 공간용 예시
# from omni.isaac.lab.envs import ManagerBasedRLEnv # Isaac Lab 환경

# --- 간단한 정책 신경망 정의 ---
class SimplePolicyNet(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, 128)
        self.fc2 = nn.Linear(128, action_dim) # 이산적 액션: 로짓(logits) 출력
        # 연속적 액션: 평균(mean)과 표준편차(std)를 별도로 출력하거나, 평균만 출력하고 고정된 std 사용

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        # 이산적 액션: 소프트맥스를 통해 확률 분포 생성
        # action_logits = self.fc2(x) 
        # return Categorical(logits=action_logits) 
        
        # 연속적 액션 (평균만 예측하는 예시, std는 고정 또는 별도 네트워크로 예측)
        action_mean = torch.tanh(self.fc2(x)) # 액션 범위를 [-1, 1]로 가정
        # action_std = torch.ones_like(action_mean) * 0.5 # 고정된 표준편차 예시
        # return Normal(action_mean, action_std)
        return action_mean # 여기서는 평균만 반환 (실제로는 분포 객체 반환이 일반적)

# --- REINFORCE 에이전트 (매우 간소화된 버전) ---
class SimpleREINFORCEAgent:
    def __init__(self, obs_dim, action_dim, lr=1e-3, gamma=0.99, device="cpu"):
        self.policy_net = SimplePolicyNet(obs_dim, action_dim).to(device)
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.gamma = gamma
        self.device = device
        self.log_probs = [] # 에피소드 동안의 로그 확률 저장
        self.rewards = []   # 에피소드 동안의 보상 저장

    def compute_action(self, observation: torch.Tensor):
        if observation.ndim == 1: # 단일 관찰 값 처리 (배치 차원 추가)
            observation = observation.unsqueeze(0)
        
        # 정책망을 통해 액션 확률 분포 또는 결정론적 액션 계산
        # action_dist = self.policy_net(observation)
        # action = action_dist.sample()
        # self.log_probs.append(action_dist.log_prob(action))
        
        # 결정론적 정책의 경우 (예: DDPG, 또는 REINFORCE의 단순화된 버전)
        action_mean = self.policy_net(observation) # 정책망이 직접 액션 평균을 출력
        # 실제 환경에서는 탐색을 위해 노이즈를 추가할 수 있음
        # action = action_mean + torch.randn_like(action_mean) * exploration_noise
        action = action_mean.squeeze(0) # 배치 차원 제거 (단일 액션 반환 시)
        
        return action # .cpu().numpy() # 필요시 NumPy로 변환

    def store_reward(self, reward):
        self.rewards.append(reward)

    def update(self):
        if not self.log_probs: # REINFORCE는 에피소드 단위 업데이트
            return

        discounted_rewards = []
        R = 0
        for r in reversed(self.rewards):
            R = r + self.gamma * R
            discounted_rewards.insert(0, R)
        
        discounted_rewards = torch.tensor(discounted_rewards, device=self.device)
        # 정규화 (선택 사항이지만 권장)
        discounted_rewards = (discounted_rewards - discounted_rewards.mean()) / (discounted_rewards.std() + 1e-8)
        
        policy_loss = []
        for log_prob, R_t in zip(self.log_probs, discounted_rewards):
            policy_loss.append(-log_prob * R_t) # 손실 함수: -log_prob(a|s) * G_t
        
        self.optimizer.zero_grad()
        policy_loss_tensor = torch.stack(policy_loss).sum()
        policy_loss_tensor.backward()
        self.optimizer.step()
        
        # 에피소드 데이터 초기화
        self.log_probs = []
        self.rewards = []

# --- 학습 루프 (Isaac Lab 환경과 연동) ---
# def train_custom_agent(env: ManagerBasedRLEnv, agent: SimpleREINFORCEAgent, num_episodes: int):
#     for episode in range(num_episodes):
#         obs_dict, _ = env.reset()
#         # Isaac Lab 환경은 관찰 값을 딕셔너리 형태로 반환할 수 있음 ('policy' 키 등)
#         # 여기서는 'policy' 키에 주 관찰 값이 있다고 가정
#         # 또한, Isaac Lab은 병렬 환경을 사용하므로, obs_dict["policy"]는 (num_envs, obs_dim) 형태일 수 있음
#         # 이 간소화된 예제에서는 num_envs=1 이라고 가정
#         current_obs = obs_dict["policy"][0].to(agent.device) # 첫 번째 환경의 관찰 값, GPU로 이동
#         done = False
#         episode_reward = 0

#         while not done:
#             # 액션 계산 (PyTorch 텐서 형태)
#             action_tensor = agent.compute_action(current_obs)
            
#             # Isaac Lab 환경은 (num_envs, action_dim) 형태의 액션을 기대함
#             # 여기서는 num_envs=1 이므로 unsqueeze(0)으로 배치 차원 추가
#             next_obs_dict, reward_tensor, done_tensor, info_dict = env.step(action_tensor.unsqueeze(0))
            
#             # 단일 환경 결과 추출
#             next_obs = next_obs_dict["policy"][0].to(agent.device)
#             reward = reward_tensor[0].item() # 스칼라 값으로 변환
#             done = done_tensor[0].item()
            
#             agent.store_reward(reward) # REINFORCE는 보상만 저장 (로그 확률은 액션 계산 시 저장)
#             current_obs = next_obs
#             episode_reward += reward
            
#             # env.render() # 필요시 렌더링 (성능 저하 유발 가능)

#         agent.update() # 에피소드 종료 후 정책 업데이트
#         print(f"Episode {episode + 1}: Total Reward = {episode_reward}")

# # --- 실제 사용 ---
# # env = ... (Isaac Lab 환경 객체 생성)
# # obs_space_shape = env.observation_space["policy"].shape 
# # action_space_shape = env.action_space.shape
# # agent = SimpleREINFORCEAgent(obs_dim=obs_space_shape[-1], action_dim=action_space_shape[-1], device=env.device)
# # train_custom_agent(env, agent, num_episodes=1000)
# # env.close()
```
**참고:** 위 REINFORCE 예제는 매우 간소화되었으며, 실제 구현에서는 로그 확률을 올바르게 계산하고 저장하며, 액션 분포(Categorical 또는 Normal)를 명시적으로 사용하는 것이 일반적입니다. 또한, Isaac Lab의 병렬 환경 처리 방식에 맞춰 관찰 값과 액션의 배치(batch) 차원을 정확히 다루어야 합니다.

## 2. 정책 및 가치 함수를 위한 신경망 아키텍처 커스터마이징

RL 에이전트의 성능은 정책(actor) 및 가치 함수(critic)를 근사하는 신경망의 아키텍처에 크게 의존합니다. 문제의 복잡성, 관찰 값의 종류(이미지, 벡터 등), 액션 공간의 특성에 따라 적절한 아키텍처를 선택하고 커스터마이징하는 것이 중요합니다.

**일반적인 아키텍처 구성 요소:**

*   **입력 계층 (Input Layers):**
    *   관찰 값의 종류에 따라 다릅니다.
        *   **벡터 관찰 값 (Vector Observations):** 일반적으로 다층 퍼셉트론(MLP: Multi-Layer Perceptron) - 완전 연결 계층(Fully Connected Layers, `nn.Linear`)의 연속으로 구성됩니다.
        *   **이미지 관찰 값 (Image Observations):** 컨볼루션 신경망(CNN: Convolutional Neural Networks - `nn.Conv2d`, `nn.MaxPool2d` 등)을 사용하여 이미지로부터 특징(feature)을 추출합니다. 추출된 특징 맵은 평탄화(flatten)되어 MLP의 입력으로 사용됩니다.
        *   **시간적(Temporal) 또는 순차적(Sequential) 데이터:** 순환 신경망(RNN: Recurrent Neural Networks - `nn.LSTM`, `nn.GRU`)을 사용하여 과거 정보의 맥락을 파악합니다.
*   **은닉 계층 (Hidden Layers):**
    *   MLP의 경우, 여러 개의 완전 연결 계층과 활성화 함수(예: ReLU, Tanh, LeakyReLU)로 구성됩니다.
    *   계층의 수(깊이)와 각 계층의 뉴런 수(너비)는 하이퍼파라미터로, 문제의 복잡성에 맞춰 조정합니다. 너무 작으면 표현력이 부족하고, 너무 크면 과적합 및 계산 비용 증가의 우려가 있습니다.
*   **출력 계층 (Output Layers):**
    *   **정책망 (Actor Network):**
        *   **이산적 액션 공간 (Discrete Action Space):** 액션의 개수만큼의 로짓(logit)을 출력하고, 이를 소프트맥스(Softmax) 함수에 통과시켜 각 액션에 대한 확률 분포를 얻습니다. (예: `Categorical` 분포)
        *   **연속적 액션 공간 (Continuous Action Space):**
            *   액션의 각 차원에 대한 평균(mean)을 출력합니다. 표준편차(standard deviation)는 고정된 값으로 사용하거나, 별도의 신경망 가지(branch) 또는 입력으로 받아 조절 가능하게 만듭니다. (예: `Normal` 또는 `MultivariateNormal` 분포)
            *   출력 값의 범위 제한을 위해 활성화 함수(예: `torch.tanh`를 사용하여 [-1, 1] 범위로 제한 후 스케일링)를 사용합니다.
    *   **가치망 (Critic Network):**
        *   상태 가치(State Value, V(s)) 또는 상태-액션 가치(State-Action Value, Q(s,a))를 나타내는 단일 스칼라 값을 출력합니다. (일반적으로 선형 계층 사용)

**커스터마이징 전략:**

*   **관찰 값 전처리:**
    *   이미지 관찰 값의 경우, 정규화(normalization), 크기 조절(resizing), 그레이스케일 변환 등의 전처리가 필요할 수 있습니다.
    *   여러 종류의 센서 데이터를 사용하는 경우(multimodal input), 각 데이터를 적절히 전처리하고 특징을 추출한 후, 이를 결합(concatenate)하여 하나의 특징 벡터로 만들어 MLP에 입력합니다.
*   **아키텍처 선택:**
    *   **단순한 문제:** 몇 개의 은닉 계층을 가진 MLP로 충분할 수 있습니다.
    *   **이미지 기반 문제 (예: 로봇 비전):** ResNet, EfficientNet 등 사전 훈련된 CNN 아키텍처의 일부를 특징 추출기로 사용하고, 그 위에 작은 MLP를 추가하는 전이 학습(Transfer Learning) 접근 방식을 고려할 수 있습니다.
    *   **주의 메커니즘 (Attention Mechanisms):** 관찰 값의 특정 부분에 더 집중해야 하는 문제(예: 여러 물체 중 특정 물체에 주의)의 경우, 트랜스포머(Transformer) 네트워크의 어텐션 레이어를 통합할 수 있습니다.
*   **활성화 함수 선택:**
    *   ReLU가 일반적으로 많이 사용되지만, LeakyReLU, Swish 등 다른 활성화 함수를 실험해 볼 수 있습니다.
    *   출력 계층에서는 액션의 범위나 가치 값의 특성에 맞는 활성화 함수를 사용해야 합니다 (예: 연속적 액션의 `tanh`).
*   **가중치 초기화 (Weight Initialization):**
    *   적절한 가중치 초기화(예: Xavier, Kaiming 초기화)는 학습 안정성과 속도에 영향을 줄 수 있습니다. PyTorch는 기본적으로 합리적인 초기화를 제공하지만, 필요에 따라 커스터마이징할 수 있습니다.
*   **정규화 기법 (Regularization Techniques):**
    *   과적합을 방지하기 위해 드롭아웃(Dropout), 배치 정규화(Batch Normalization), 가중치 감쇠(Weight Decay) 등을 사용할 수 있습니다. 특히 배치 정규화는 학습을 안정화하고 수렴 속도를 높이는 데 도움이 될 수 있습니다.
*   **Actor-Critic 아키텍처 공유 vs. 분리:**
    *   Actor-Critic 계열 알고리즘(A2C, A3C, PPO, SAC 등)에서 액터와 크리틱 네트워크가 입력 계층이나 일부 은닉 계층을 공유할지, 아니면 완전히 분리된 네트워크를 사용할지 결정할 수 있습니다.
    *   계층 공유는 파라미터 수를 줄이고 학습 효율을 높일 수 있지만, 두 네트워크의 학습 목표가 달라 서로 간섭을 일으킬 수도 있습니다.

**Isaac Lab에서의 구현:**

*   사용자 정의 신경망 모델은 PyTorch의 `nn.Module`을 상속받아 Python 클래스로 쉽게 정의할 수 있습니다.
*   Skrl과 같은 라이브러리를 사용하는 경우, 해당 라이브러리가 제공하는 모델 생성 인터페이스나 사용자 정의 모델 등록 방법을 따라야 합니다. (예: skrl은 모델을 딕셔너리 형태로 정의하거나, 직접 `nn.Module` 객체를 전달하는 방식을 지원)
*   커스텀 알고리즘을 직접 구현하는 경우, 위에서 정의한 PyTorch 모델 객체를 생성하여 학습 루프 내에서 사용합니다.

**예시: Skrl에서 MLP 아키텍처 커스터마이징 (개념)**

```python
# from skrl.models.torch import Model, GaussianMixin, DeterministicMixin # skrl 모델 클래스

# # skrl에서 MLP 모델을 정의하는 방식 (딕셔셔너리 또는 직접 클래스 상속)
# # 1. 딕셔너리 방식 (정책 - Gaussian Policy)
# policy_cfg = {
#     "network": { # skrl.resources.modules.torch.MLP 사용
#         "layers": [ # 은닉 계층 정의 (입력/출력 크기는 skrl이 자동으로 설정)
#             {"units": 256, "activation": "relu"},
#             {"units": 128, "activation": "relu"}
#         ]
#     },
#     "output_scale": 1.0, # 액션 스케일링
#     "log_std_min": -20,  # 로그 표준편차 최소값
#     "log_std_max": 2     # 로그 표준편차 최대값
# }

# # 2. 직접 클래스 상속 방식 (더 세밀한 제어 가능)
# class CustomPolicy(GaussianMixin, Model): # GaussianMixin은 연속적 액션, 확률적 정책
#     def __init__(self, observation_space, action_space, device, **kwargs):
#         Model.__init__(self, observation_space, action_space, device, **kwargs)
#         GaussianMixin.__init__(self, **kwargs) # log_std_min, log_std_max 등 처리

#         obs_size = observation_space.shape[0]
#         action_size = action_space.shape[0]

#         self.net = nn.Sequential(
#             nn.Linear(obs_size, 256),
#             nn.ReLU(),
#             nn.Linear(256, 128),
#             nn.ReLU()
#         )
#         self.mean_layer = nn.Linear(128, action_size)
#         # 로그 표준편차를 위한 파라미터 (학습 가능하게)
#         self.log_std_parameter = nn.Parameter(torch.zeros(action_size)) 
        
#     def compute(self, inputs, role): # Model 클래스의 추상 메서드 구현
#         x = self.net(inputs["states"]) # 관찰 값은 inputs["states"]로 전달됨
#         # 평균값 출력, tanh를 통해 [-1, 1] 범위로 제한 후 스케일링
#         # (GaussianMixin이 output_scale을 사용하여 자동 스케일링 해줌)
#         mean = torch.tanh(self.mean_layer(x)) 
        
#         # 로그 표준편차 반환 (GaussianMixin이 이를 사용하여 분포 생성)
#         log_std = self.log_std_parameter.expand_as(mean) 
#         return mean, log_std, {} # 마지막은 추가 정보 (비워둠)

# # 위에서 정의한 모델을 skrl 에이전트 설정 시 전달
# # agent_cfg["policy_model"] = CustomPolicy 
# # agent_cfg["policy_model_kwargs"] = {"log_std_min": -5, "log_std_max": 2}
```

RL 에이전트의 핵심은 결국 정책과 가치 함수를 표현하는 신경망이므로, 문제의 특성에 맞는 적절한 아키텍처를 설계하고 실험하는 것은 성공적인 RL 애플리케이션 개발에 있어 매우 중요한 단계입니다. Isaac Lab은 이러한 유연성을 제공하여 사용자가 다양한 아이디어를 시도해 볼 수 있도록 지원합니다.
