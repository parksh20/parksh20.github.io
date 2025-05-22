# Isaac Lab 고급 토픽: 커리큘럼 학습 (Curriculum Learning)

강화학습(RL) 에이전트가 복잡한 작업을 처음부터 학습하는 것은 매우 어려울 수 있습니다. 마치 사람이 어려운 문제를 풀기 전에 쉬운 문제부터 단계적으로 학습하는 것처럼, RL 에이전트도 점진적으로 작업의 난이도를 높여가며 학습할 때 더 효율적으로 좋은 성능을 달성할 수 있습니다. 이러한 학습 전략을 커리큘럼 학습(Curriculum Learning)이라고 합니다.

이 섹션에서는 커리큘럼 학습의 개념과 장점, 그리고 Isaac Lab 환경에서 이를 구현하기 위한 다양한 전략과 방법에 대해 설명합니다.

## 1. 커리큘럼 학습 전략 및 장점

**커리큘럼 학습(Curriculum Learning)이란?**

커리큘럼 학습은 RL 에이전트에게 처음에는 쉬운 작업(또는 쉬운 환경)을 제공하고, 에이전트의 학습 진행도에 따라 점차 작업의 난이도를 높여나가는 전략입니다. 목표는 에이전트가 학습 초기에 의미 있는 보상(reward)을 쉽게 얻도록 하여 학습을 가속화하고, 최종적으로는 어려운 목표 작업에서도 좋은 성능을 내도록 유도하는 것입니다.

**커리큘럼 학습의 주요 구성 요소:**

1.  **난이도 측정 (Difficulty Measure):** 작업의 어려움을 정량적으로 측정할 수 있는 지표입니다. 예를 들어, 목표까지의 거리, 장애물의 수, 필요한 정확도 등이 될 수 있습니다.
2.  **커리큘럼 스케줄 (Curriculum Schedule):** 언제, 어떻게 작업의 난이도를 높일지를 결정하는 계획입니다. 에이전트의 성능(예: 평균 보상, 성공률)에 기반하여 자동으로 난이도를 조절하거나, 미리 정의된 순서에 따라 난이도를 변경할 수 있습니다.
3.  **작업 공간 또는 환경 파라미터 (Task Space or Environment Parameters):** 난이도를 조절하기 위해 변경할 수 있는 환경의 요소들입니다. (예: 목표물의 위치 범위, 로봇의 초기 조건, 물리 파라미터의 무작위화 범위 등)

**커리큘럼 학습의 장점:**

*   **학습 속도 향상 (Faster Convergence):**
    *   에이전트는 학습 초기에 쉬운 작업을 통해 빠르게 성공 경험을 쌓고 유의미한 정책을 학습할 수 있습니다. 이는 무작위 탐색(random exploration)에 소요되는 시간을 줄여 전체적인 학습 속도를 높입니다.
*   **더 나은 최종 성능 (Improved Final Performance):**
    *   점진적인 난이도 상승은 에이전트가 지역 최적해(local optima)에 빠지는 것을 방지하고, 더 넓은 상태 공간(state space)을 효과적으로 탐색하여 최종적으로 더 우수한 정책을 찾도록 도와줍니다.
*   **복잡한 작업 학습 가능성 증대 (Increased Likelihood of Learning Complex Tasks):**
    *   매우 희소한 보상(sparse reward)을 가지거나 탐색 공간이 매우 넓은 어려운 작업의 경우, 처음부터 해당 작업을 학습하는 것은 거의 불가능할 수 있습니다. 커리큘럼 학습은 이러한 문제에 대한 효과적인 해결책을 제공합니다.
*   **강인성(Robustness) 향상에 기여:**
    *   다양한 난이도의 환경에 점진적으로 노출됨으로써, 예기치 않은 상황이나 변화에 대해 더 잘 일반화되고 강인한 정책을 학습하는 데 도움이 될 수 있습니다. (도메인 무작위화와 함께 사용될 때 시너지 효과)

## 2. Isaac Lab 환경에서 커리큘럼 학습 구현

Isaac Lab은 환경 설정과 Python 스크립팅을 통해 커리큘럼 학습을 구현할 수 있는 유연성을 제공합니다. 핵심은 에이전트의 학습 진행 상황에 따라 환경의 특정 파라미터를 동적으로 변경하는 로직을 추가하는 것입니다.

**구현 전략 및 방법:**

1.  **작업 난이도와 관련된 환경 파라미터 식별:**
    *   먼저, 어떤 환경 파라미터를 변경하여 작업의 난이도를 조절할 수 있을지 결정해야 합니다.
    *   **예시:**
        *   **목표의 근접성/단순성:**
            *   로봇 팔 작업: 초기에는 물체를 그리퍼 바로 앞에 놓고, 점차 물체와의 거리나 방향을 무작위화합니다.
            *   내비게이션 작업: 초기에는 목표 지점을 로봇 가까이에 설정하고, 점차 목표 지점까지의 거리를 늘리거나 경로 상에 장애물을 추가합니다.
        *   **초기 상태의 안정성:**
            *   로봇의 초기 자세를 안정적인 상태에서 시작하여 점차 불안정한 초기 자세(예: 약간 기울어진 상태)를 허용합니다.
        *   **물리적 외란의 강도:**
            *   초기에는 외부 힘이나 토크를 가하지 않거나 약하게 하고, 점차 그 강도나 빈도를 높입니다. (도메인 무작위화의 범위 조절과 유사)
        *   **센서 노이즈 수준:**
            *   초기에는 깨끗한 센서 데이터를 제공하고, 점차 노이즈 수준을 높입니다.
        *   **도움(Reward Shaping)의 강도:**
            *   초기에는 목표 달성을 돕는 보조적인 보상(auxiliary reward)을 많이 주고, 학습이 진행됨에 따라 점차 줄여나가 최종 목표 보상에 집중하도록 합니다.

2.  **에이전트 성능 모니터링:**
    *   커리큘럼의 다음 단계로 넘어갈 시점을 결정하기 위해 에이전트의 성능을 지속적으로 모니터링해야 합니다.
    *   **주요 성능 지표:**
        *   **평균 에피소드 보상 (Average Episodic Reward):** 가장 일반적으로 사용되는 지표입니다.
        *   **작업 성공률 (Success Rate):** 특정 기준(예: 목표 도달, 물체 집기 성공)을 만족하는 에피소드의 비율입니다.
        *   **에피소드 길이 (Episode Length):** 작업을 완료하는 데 걸리는 시간 또는 스텝 수.
    *   Isaac Lab의 학습 프레임워크(예: skrl과 연동 시)는 이러한 통계들을 로깅하고 접근할 수 있는 방법을 제공합니다. (TensorBoard 등으로 시각화 가능)

3.  **커리큘럼 스케줄링 로직 구현:**
    *   에이전트의 성능 지표가 특정 임계값(threshold)에 도달하면 환경 파라미터를 변경하여 난이도를 높이는 로직을 구현합니다.
    *   **구현 위치:**
        *   **학습 스크립트 (예: `train.py`):** 메인 학습 루프에서 주기적으로 에이전트 성능을 확인하고, 조건 충족 시 환경 객체의 관련 파라미터 변경 함수를 호출합니다.
        *   **환경 클래스 내부:** 환경 클래스 자체에 커리큘럼 관리 로직을 포함시키고, `step` 또는 `reset` 함수 내에서 난이도를 업데이트할 수 있습니다.
        *   **Orbit의 이벤트 콜백:** 특정 이벤트(예: 정해진 학습 스텝 수 도달) 발생 시 난이도를 업데이트하는 콜백을 등록할 수 있습니다.

    ```python
    # 예시: 학습 스크립트 내에서 커리큘럼 관리 (개념 코드)
    # from omni.isaac.lab.envs import ManagerBasedRLEnv # 실제 환경 클래스 임포트
    
    # class CurriculumManager:
    #     def __init__(self, env: ManagerBasedRLEnv, initial_difficulty: float = 0.1):
    #         self.env = env
    #         self.current_difficulty = initial_difficulty
    #         self.reward_threshold_high = 200 # 다음 단계로 넘어갈 보상 임계값 (높음)
    #         self.reward_threshold_low = 50   # 이전 단계로 돌아갈 보상 임계값 (낮음, 선택적)
    #         self.difficulty_increment = 0.1
    #         self.max_difficulty = 1.0
    #         self.min_difficulty = 0.1
            
    #         # 환경에 난이도 설정 인터페이스가 있다고 가정
    #         if hasattr(self.env, "set_task_difficulty"):
    #             self.env.set_task_difficulty(self.current_difficulty)

    #     def update_curriculum(self, average_reward: float):
    #         if not hasattr(self.env, "set_task_difficulty"):
    #             return

    #         if average_reward > self.reward_threshold_high and self.current_difficulty < self.max_difficulty:
    #             self.current_difficulty = min(self.current_difficulty + self.difficulty_increment, self.max_difficulty)
    #             self.env.set_task_difficulty(self.current_difficulty)
    #             print(f"Curriculum: Increased difficulty to {self.current_difficulty:.2f}")
    #         # 선택적: 성능이 너무 낮아지면 난이도를 다시 낮추는 로직
    #         # elif average_reward < self.reward_threshold_low and self.current_difficulty > self.min_difficulty:
    #         #     self.current_difficulty = max(self.current_difficulty - self.difficulty_increment, self.min_difficulty)
    #         #     self.env.set_task_difficulty(self.current_difficulty)
    #         #     print(f"Curriculum: Decreased difficulty to {self.current_difficulty:.2f}")

    # # --- 학습 루프 내에서 사용 ---
    # # env = ... (Isaac Lab 환경 객체)
    # # curriculum_manager = CurriculumManager(env)
    # # logged_rewards = []
    
    # # for i in range(num_learning_iterations):
    # #     # ... (에이전트 학습 및 에피소드 실행) ...
    # #     # new_rewards = ... (최근 에피소드들의 보상 수집)
    # #     logged_rewards.extend(new_rewards)
        
    # #     if (i + 1) % update_curriculum_interval == 0: # 일정 간격마다 커리큘럼 업데이트
    # #         if logged_rewards:
    # #             average_reward = sum(logged_rewards) / len(logged_rewards)
    # #             curriculum_manager.update_curriculum(average_reward)
    # #             logged_rewards = [] # 보상 기록 초기화
    ```

4.  **환경 파라미터 동적 변경 인터페이스 제공:**
    *   커리큘럼 스케줄러가 환경의 난이도를 실제로 변경할 수 있도록, Isaac Lab 환경 클래스 내에 관련 파라미터를 수정하는 메서드를 구현해야 합니다.
    *   **예시 (환경 클래스 내):**
        ```python
        # Isaac Lab 환경 클래스 내 (예: MyCustomEnv)
        # def set_task_difficulty(self, difficulty_level: float):
        #     """
        #     입력받은 난이도 수준(예: 0.0 ~ 1.0)에 따라 환경 파라미터를 조정합니다.
        #     """
        #     self.current_difficulty_level = difficulty_level # 현재 난이도 저장
            
        #     # 예시: 목표까지의 최대 거리 조절
        #     # self.task_cfg.target_distance_range = [0.5, 0.5 + 2.0 * difficulty_level] 
            
        #     # 예시: 로봇 초기 상태의 무작위화 범위 조절
        #     # self.robot_cfg.initial_pose_noise_std = 0.01 + 0.2 * difficulty_level
            
        #     # 예시: 도메인 무작위화 강도 조절
        #     # if hasattr(self.randomization_manager, "set_randomization_strength"):
        #     #    self.randomization_manager.set_randomization_strength(difficulty_level)
            
        #     print(f"Environment difficulty set to: {difficulty_level:.2f}")
        #     # 변경된 파라미터는 다음 환경 리셋 시 적용되도록 하거나,
        #     # 즉시 적용 가능한 경우 관련 로직 호출
        ```
    *   `set_task_difficulty` 메서드는 난이도 수준(예: 0.0에서 1.0 사이의 값)을 입력으로 받아, 해당 수준에 맞게 내부적으로 여러 환경 파라미터(예: 목표 거리, 장애물 밀도, 물리 효과의 강도)를 조정합니다.
    *   변경될 수 있는 파라미터들은 환경 설정 파일(TOML)에서 기본값과 범위를 정의하고, 커리큘럼 로직이 이 값을 덮어쓰거나 스케일링하도록 설계할 수 있습니다.

**커리큘럼 학습과 도메인 무작위화의 관계:**

*   커리큘럼 학습과 도메인 무작위화는 상호 보완적으로 사용될 수 있습니다.
*   예를 들어, 학습 초기에는 도메인 무작위화의 범위를 좁게(또는 비활성화)하고 작업 자체의 난이도를 낮게 설정합니다.
*   에이전트가 기본적인 작업을 학습함에 따라, 작업의 복잡성을 높이는 동시에 도메인 무작위화의 범위도 점차 넓혀 정책의 일반화 성능과 강인성을 함께 향상시킬 수 있습니다.
*   즉, 커리큘럼의 한 축으로 "작업의 구조적 난이도"를, 다른 축으로 "환경의 다양성/불확실성 수준(DR 강도)"을 고려할 수 있습니다.

**고려 사항:**

*   **커리큘럼 설계의 어려움:** 효과적인 커리큘럼(어떤 파라미터를, 어떤 순서로, 언제 변경할 것인가)을 설계하는 것은 시행착오가 필요하며, 해당 작업과 환경에 대한 깊은 이해가 요구됩니다.
*   **성능 지표의 안정성:** 에이전트의 성능 지표는 학습 과정에서 변동성이 클 수 있습니다. 단기적인 성능 변화에 너무 민감하게 반응하여 난이도를 자주 바꾸기보다는, 이동 평균(moving average) 등을 사용하여 안정화된 성능 지표를 기준으로 판단하는 것이 좋습니다.
*   **과도한 단순화 방지:** 학습 초기에 작업을 너무 쉽게 만들면, 에이전트가 최종 목표 작업을 해결하는 데 필요한 핵심적인 어려움을 경험하지 못하고 잘못된 방향으로 학습할 수 있습니다.
*   **자동 커리큘럼 학습 (Automatic Curriculum Learning):** 최근 연구에서는 에이전트의 학습 상태에 맞춰 자동으로 커리큘럼을 생성하거나 조절하는 방법(예: 목표 생성형 적대 신경망, 점진적 난이도 추정)들도 활발히 연구되고 있습니다. 이는 수동 커리큘럼 설계의 어려움을 줄여줄 수 있는 잠재력을 가집니다.

커리큘럼 학습은 복잡하고 어려운 로봇 작업을 강화학습으로 해결하고자 할 때 매우 유용한 전략입니다. Isaac Lab의 유연한 환경 구성 기능을 활용하여, 에이전트가 점진적으로 성장하며 최종 목표를 달성할 수 있도록 효과적인 학습 경로를 설계해 보시기 바랍니다.
