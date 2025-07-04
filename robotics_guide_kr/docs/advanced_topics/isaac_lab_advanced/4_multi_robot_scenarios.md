# Isaac Lab 고급 토픽: 멀티 로봇 시나리오

단일 로봇 시스템을 넘어 여러 로봇이 동시에 작업을 수행하거나 상호작용하는 멀티 로봇 시스템(Multi-Robot Systems, MRS)은 로봇 공학 연구의 중요한 분야입니다. MRS는 물류 창고 자동화, 탐사, 수색 및 구조, 협동 제조 등 다양한 애플리케이션에서 단일 로봇보다 더 높은 효율성, 강인성, 확장성을 제공할 수 있습니다.

Isaac Lab은 여러 로봇을 동일한 환경 내에 동시에 시뮬레이션하고, 이들의 행동을 개별적으로 또는 통합적으로 제어하며, 멀티 에이전트 강화학습(Multi-Agent Reinforcement Learning, MARL)을 적용할 수 있는 강력한 기반을 제공합니다.

이 섹션에서는 Isaac Lab에서 멀티 로봇 환경을 설정하는 방법, 협업 및 경쟁 작업을 구현하기 위한 아이디어, 그리고 멀티 로봇 시나리오와 관련된 주요 과제 및 해결 방안에 대해 논의합니다.

## 1. 멀티 에이전트 환경 설정

Isaac Lab에서 멀티 로봇 환경을 설정하는 것은 기본적으로 단일 로봇 환경 설정의 확장입니다. 핵심은 여러 로봇 에셋을 씬(scene)에 로드하고, 각 로봇에 대한 관찰(observation)과 액션(action)을 개별적으로 또는 그룹으로 처리하며, 작업(task)과 보상(reward)을 멀티 로봇 관점에서 정의하는 것입니다.

**설정 단계 및 고려 사항:**

1.  **로봇 에셋 준비 및 로드:**
    *   시나리오에 사용할 각 로봇의 USD 에셋을 준비합니다. 동일한 종류의 로봇 여러 대를 사용하거나, 서로 다른 종류의 로봇들을 조합할 수 있습니다.
    *   Isaac Lab 환경의 씬 구성 부분(일반적으로 환경 설정 파일이나 Python 스크립트 내 `_create_scene` 메서드 등)에서 각 로봇 에셋을 원하는 초기 위치와 방향으로 씬에 로드합니다.
    *   각 로봇 프림에 고유한 이름을 부여하여 구분할 수 있도록 합니다. (예: `/World/robot_0`, `/World/robot_1`, `/World/quadcopter_A`, `/World/manipulator_B`)

2.  **로봇 매니저(RobotManager) 또는 에셋 매니저(AssetManager) 활용:**
    *   Isaac Lab (Orbit)의 `RobotManager` 또는 `AssetManager`를 사용하여 여러 로봇을 효율적으로 관리할 수 있습니다.
    *   설정 파일(TOML 또는 Python)에서 각 로봇의 이름, USD 경로, 초기 상태, 제어 인터페이스 등을 정의하고, 매니저가 이를 바탕으로 로봇들을 초기화하고 씬에 추가하도록 합니다.
    *   **예시 (환경 설정 파일 - TOML 개념):**
        ```toml
        # [[scene.robot]] # 로봇 그룹 또는 개별 로봇 정의
        # name = "franka_0"
        # usd_path = "/Isaac/Robots/Franka/franka_instanceable.usd"
        # position = [0.0, -0.5, 0.0]
        # # ... 기타 설정 ...
        
        # [[scene.robot]]
        # name = "franka_1"
        # usd_path = "/Isaac/Robots/Franka/franka_instanceable.usd"
        # position = [0.0, 0.5, 0.0]
        # # ... 기타 설정 ...
        
        # [[scene.my_quadcopters]] # 그룹으로 정의하고 수를 지정할 수도 있음
        # usd_path = "/Isaac/Robots/Quadrotor/quadrotor_instanceable.usd"
        # num_robots = 5
        # spawn_locations = [ # 각 로봇의 초기 위치 리스트
        #     [1.0, 0.0, 0.5], [1.0, 1.0, 0.5], ... 
        # ]
        # name_prefix = "drone" # 로봇 이름 접두사 (예: drone_0, drone_1)
        ```

3.  **관찰 공간(Observation Space) 및 액션 공간(Action Space) 정의:**
    *   **중앙 집중식(Centralized) 접근:** 하나의 RL 에이전트가 모든 로봇을 제어하는 경우, 관찰 공간은 모든 로봇의 관찰 값을 결합(concatenate)한 형태가 되고, 액션 공간도 모든 로봇의 액션을 결합한 형태가 됩니다. (에이전트 수가 적거나 로봇 간 상호작용이 매우 밀접할 때)
    *   **분산형(Decentralized) 또는 부분 관찰(Partially Observable) 접근:** 각 로봇이 독립적인 에이전트에 의해 제어되거나, 각 에이전트가 자신의 로봇과 주변 일부 정보만 관찰하는 경우입니다.
        *   이 경우, 각 에이전트(또는 로봇)별로 개별적인 관찰 공간과 액션 공간을 정의해야 합니다.
        *   Isaac Lab 환경은 일반적으로 모든 로봇/에이전트에 대한 관찰과 액션을 하나의 큰 텐서(배치 차원 포함)로 묶어 처리하는 경우가 많으므로, MARL 알고리즘과의 연동 시 이 부분을 어떻게 분리하고 매핑할지 고려해야 합니다.
    *   **관찰 값 구성:** 각 로봇은 자신의 상태(예: 조인트 각도, 속도, 포즈) 외에도, 다른 로봇의 상태, 목표물의 위치, 장애물 정보 등을 관찰 값에 포함할 수 있습니다. 어떤 정보를 공유하고 관찰할지가 MARL의 핵심 설계 요소입니다.

4.  **액션 적용 및 관찰 값 수집:**
    *   환경의 `step()` 함수에서는 모든 에이전트로부터 액션을 받아 각 해당 로봇에 적용해야 합니다.
    *   마찬가지로, `step()` 함수는 각 로봇(또는 에이전트)에 해당하는 다음 관찰 값을 계산하여 반환해야 합니다.
    *   Isaac Lab의 환경은 일반적으로 `num_envs` (병렬 실행 환경 수) 외에, 환경 내 에이전트 수(`num_agents_per_env` 또는 유사한 개념)를 고려하여 관찰 및 액션 텐서의 차원을 설정합니다. 예를 들어, 액션 텐서는 `(num_envs, num_agents_per_env, action_dim_per_agent)` 형태가 될 수 있습니다.

5.  **보상 함수(Reward Function) 설계:**
    *   MARL에서 보상 함수 설계는 매우 중요하며 다양한 방식이 가능합니다.
        *   **개별 보상 (Individual Rewards):** 각 에이전트는 자신의 행동과 목표 달성도에 따라 개별적인 보상을 받습니다. (경쟁적 또는 독립적 작업에 적합)
        *   **팀 보상 (Team Rewards / Shared Reward):** 모든 에이전트가 동일한 보상(팀 전체의 성과)을 공유합니다. (완전 협동 작업에 적합)
        *   **혼합 보상 (Mixed Rewards):** 개별 보상과 팀 보상을 조합하여 사용합니다.
    *   보상 함수는 각 로봇의 상태, 목표 달성 여부, 다른 로봇과의 상호작용(예: 충돌, 협력 행동) 등을 고려하여 설계됩니다.

## 2. 멀티 로봇 협업 및 경쟁 작업 구현

Isaac Lab 환경에서 다양한 멀티 로봇 협업 및 경쟁 작업을 시나리오로 구현하고 학습시킬 수 있습니다.

**협업 작업 (Collaborative Tasks) 예시:**

*   **협동 물체 운반 (Cooperative Object Transportation):**
    *   두 대 이상의 로봇 팔 또는 이동 로봇이 하나의 큰 물체를 함께 들어 올리고 목표 지점까지 운반합니다.
    *   **과제:** 로봇 간의 힘 분배, 동기화된 움직임, 충돌 회피, 물체 안정성 유지.
    *   **관찰 값:** 각 로봇의 상태, 물체의 포즈 및 속도, 로봇 간 상대 위치, 그리퍼 센서 데이터.
    *   **보상:** 물체를 성공적으로 운반한 경우 큰 보상, 물체의 안정성 유지, 목표 지점 근접도, 시간 패널티 등.
*   **협동 탐색 및 매핑 (Collaborative Exploration and Mapping):**
    *   여러 대의 이동 로봇이 미지의 환경을 함께 탐색하여 지도를 작성하고 특정 목표물을 찾습니다.
    *   **과제:** 탐색 영역 분담, 정보 공유(예: 발견한 지도 조각, 목표물 위치), 중복 탐색 최소화.
    *   **관찰 값:** 각 로봇의 위치, 센서 데이터(라이다, 카메라), 현재까지의 개인/공유 지도.
    *   **보상:** 새로 탐색한 영역의 크기, 목표물 발견, 지도 완성도, 시간 효율성.
*   **분산된 작업 할당 및 수행 (Distributed Task Allocation and Execution):**
    *   여러 개의 작업이 주어졌을 때, 각 로봇이 효율적으로 작업을 분담하고 수행합니다. (예: 여러 위치에 있는 물건 수거)
    *   **과제:** 최적의 작업 할당, 로봇 간 경로 충돌 회피, 작업 완료 시간 최소화.
*   **대형 조립 작업 (Collaborative Assembly):**
    *   여러 로봇 팔이 각자 다른 부품을 다루거나 특정 조립 단계를 수행하여 하나의 큰 제품을 완성합니다.

**경쟁 작업 (Competitive Tasks) 예시:**

*   **자원 경쟁 (Resource Competition):**
    *   한정된 자원(예: 특정 위치의 에너지원, 수집해야 할 아이템)을 두고 여러 로봇이 경쟁합니다.
    *   **과제:** 효율적인 자원 확보 전략, 다른 로봇의 방해 또는 차단.
    *   **보상:** 획득한 자원의 양에 따라 개별 보상.
*   **추격-회피 (Pursuit-Evasion):**
    *   일부 로봇(추격자)이 다른 로봇(회피자)을 잡으려고 하고, 회피자는 잡히지 않으려고 합니다.
    *   **관찰 값:** 상대방 로봇의 위치 및 속도, 자신의 상태.
    *   **보상:** 추격자는 잡으면 보상, 회피자는 잡히지 않으면(또는 오래 버티면) 보상.
*   **로봇 스포츠 (Robot Sports):**
    *   두 팀으로 나뉜 로봇들이 축구나 하키와 같은 스포츠 게임을 합니다. (예: 로봇 축구에서 골 넣기)

**구현 시 고려 사항:**

*   **통신 (Communication):** 협업 작업의 경우, 로봇(에이전트) 간의 명시적인 통신 메커니즘을 모델링할지 여부를 결정해야 합니다. (예: 통신 채널을 통해 메시지 교환, 또는 다른 에이전트의 행동/상태 관찰을 통한 암묵적 통신)
*   **관찰의 범위 (Observation Scope):** 각 에이전트가 전체 환경 상태(Global State)를 모두 관찰할 수 있는지, 아니면 자신의 로컬 환경(Local Observation)과 주변 일부 에이전트 정보만 관찰할 수 있는지(Partial Observability) 결정해야 합니다. 부분 관찰 가능성은 실제 환경과 더 유사하지만 MARL 문제를 더 어렵게 만듭니다.
*   **액션 동기화:** 모든 로봇이 동시에 액션을 수행하는지, 아니면 순차적으로 또는 비동기적으로 액션을 수행하는지 결정합니다. Isaac Lab의 병렬 환경은 기본적으로 동기식 액션 실행을 가정하는 경우가 많습니다.

## 3. 관련 과제 및 해결 방안 (MARL Challenges)

멀티 로봇 시나리오, 특히 MARL을 적용할 때는 단일 에이전트 RL에 비해 몇 가지 추가적인 어려움이 따릅니다.

*   **비정상성 문제 (Non-Stationarity):**
    *   각 에이전트의 입장에서 볼 때, 다른 에이전트들도 동시에 학습하고 정책을 변경하므로 환경이 비정상적(non-stationary)으로 보이게 됩니다. 즉, 한 에이전트의 최적 정책이 다른 에이전트의 정책 변화에 따라 계속 바뀔 수 있습니다.
    *   **해결 방안 아이디어:**
        *   **중앙 집중식 학습, 분산형 실행 (Centralized Training with Decentralized Execution, CTDE):** 학습 시에는 모든 에이전트의 관찰과 액션을 사용하는 중앙 컨트롤러(크리틱)를 두어 안정적인 학습을 유도하고, 실행 시에는 각 에이전트가 자신의 로컬 관찰만으로 액션을 결정하도록 합니다. (예: MADDPG, COMA)
        *   다른 에이전트의 정책이나 의도를 추론하는 모델 사용.
*   **신용 할당 문제 (Credit Assignment):**
    *   팀 보상을 사용하는 경우, 팀 전체의 성공 또는 실패에 각 에이전트가 얼마나 기여했는지 파악하기 어렵습니다.
    *   **해결 방안 아이디어:**
        *   차이 보상(Difference Rewards): 팀 전체 보상에서 해당 에이전트가 없었을 경우의 보상을 뺀 값을 개별 보상으로 사용.
        *   Counterfactual Multi-Agent Policy Gradients (COMA)와 같이 각 에이전트의 기여도를 추정하는 방법 사용.
*   **확장성 문제 (Scalability):**
    *   에이전트 수가 증가함에 따라 관찰 공간, 액션 공간, 그리고 상호작용의 복잡성이 기하급수적으로 증가할 수 있습니다.
    *   **해결 방안 아이디어:**
        *   파라미터 공유(Parameter Sharing): 모든 (또는 일부) 에이전트가 동일한 정책 네트워크 파라미터를 공유하여 학습할 파라미터 수를 줄입니다.
        *   Mean Field 이론, Graph Neural Network 등을 사용하여 에이전트 간 상호작용을 효율적으로 모델링.
*   **탐색의 어려움 (Exploration Challenges):**
    *   여러 에이전트가 동시에 탐색을 수행해야 하므로 효과적인 탐색 전략 수립이 더 어렵습니다.
*   **부분 관찰 가능성 (Partial Observability):**
    *   실제 환경에서는 각 로봇이 전체 환경 상태를 알 수 없는 경우가 많습니다. 이는 POMDP(Partially Observable Markov Decision Process) 문제로 이어지며, 상태 추정(state estimation)이나 메모리(memory)를 가진 에이전트(예: RNN 기반 정책)가 필요할 수 있습니다.

Isaac Lab은 이러한 멀티 로봇 및 MARL 연구를 위한 유연하고 강력한 시뮬레이션 플랫폼을 제공합니다. 사용자는 환경 구성, 로봇 정의, 관찰/액션 공간 설계, 보상 함수 커스터마이징 등을 통해 다양한 멀티 로봇 시나리오를 만들고, 여기에 최신 MARL 알고리즘을 적용하여 복잡한 협업 및 경쟁 행동을 학습시킬 수 있습니다. 도전적인 분야이지만, 그만큼 흥미로운 연구 주제와 애플리케이션 가능성을 제공합니다.
