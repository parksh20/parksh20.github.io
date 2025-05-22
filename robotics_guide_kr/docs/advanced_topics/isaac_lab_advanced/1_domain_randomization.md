# Isaac Lab 고급 토픽: 도메인 무작위화 (Domain Randomization)

강화학습(RL)에서 Sim-to-Real 이전(transfer)은 매우 중요한 과제입니다. 시뮬레이션에서 학습된 정책(policy)이 실제 로봇에서도 잘 작동하도록 하려면, 시뮬레이션 환경과 실제 환경 간의 차이, 즉 "현실 격차(reality gap)"를 줄여야 합니다. 도메인 무작위화(Domain Randomization, DR)는 이러한 현실 격차를 극복하고 학습된 정책의 강인성(robustness)을 높이는 데 사용되는 강력한 기법입니다.

이 섹션에서는 도메인 무작위화의 개념과 중요성, Isaac Lab에서 이를 구현하는 다양한 방법, 그리고 관련 설정 및 예제에 대해 심층적으로 알아봅니다.

## 1. 도메인 무작위화의 개념 및 중요성

**도메인 무작위화(Domain Randomization)란?**

도메인 무작위화는 시뮬레이션 환경의 다양한 파라미터들을 매 학습 에피소드(episode) 또는 스텝(step)마다 무작위로 변경하는 기법입니다. 이러한 파라미터에는 물리적 속성(예: 질량, 마찰력, 모터 토크), 시각적 요소(예: 텍스처, 조명, 카메라 위치), 센서 노이즈 등이 포함될 수 있습니다.

아이디어는 RL 에이전트가 매우 다양한 시뮬레이션 환경에 노출되도록 하여, 특정 시뮬레이션 환경에 과적합(overfitting)되는 것을 방지하고, 실제 환경에서 마주칠 수 있는 불확실성과 변화에 대해 더욱 강인한 정책을 학습하도록 유도하는 것입니다. 즉, 에이전트가 "어떤 환경에서도 잘 작동하는" 일반화된 능력을 키우도록 돕는 것입니다.

**왜 중요한가? (Importance of Domain Randomization)**

*   **Sim-to-Real 이전 성능 향상:**
    *   실제 세계는 예측 불가능하고 다양한 변화 요소를 가지고 있습니다. 시뮬레이션이 아무리 정교하더라도 실제 세계의 모든 측면을 완벽하게 모델링하는 것은 불가능합니다.
    *   도메인 무작위화를 통해 에이전트는 다양한 변화에 미리 적응하는 방법을 학습하므로, 시뮬레이션에서 학습한 정책이 실제 로봇에서도 더 잘 작동할 가능성이 높아집니다.
*   **정책의 강인성(Robustness) 증대:**
    *   다양한 물리적, 시각적 조건에서 학습된 정책은 예상치 못한 외란(perturbation)이나 센서 노이즈, 모델링 오류 등에 대해 더 강인한 대처 능력을 보입니다.
*   **시스템 식별(System Identification)의 어려움 회피:**
    *   실제 로봇과 환경의 모든 물리 파라미터를 정확하게 측정하고 시뮬레이션에 반영하는 것은 매우 어렵고 시간이 많이 소요되는 작업입니다.
    *   도메인 무작위화는 이러한 정확한 시스템 식별의 필요성을 줄여줍니다. 대신, 파라미터의 가능한 범위를 추정하고 그 범위 내에서 무작위화를 수행합니다.
*   **과적합 방지:**
    *   단일 또는 소수의 고정된 시뮬레이션 환경에서만 학습하면 에이전트가 해당 특정 환경에만 최적화되어 새로운 환경에서는 성능이 저하될 수 있습니다. 도메인 무작위화는 이러한 과적합을 방지합니다.

## 2. Isaac Lab에서 도메인 무작위화 구현 방법

Isaac Lab은 환경 설정과 Python 스크립팅을 통해 다양한 방식으로 도메인 무작위화를 구현할 수 있는 유연성을 제공합니다. Orbit 프레임워크의 이벤트 기반 시스템과 매니저(Manager)들을 활용하여 특정 시점에 무작위화 로직을 실행할 수 있습니다.

**주요 무작위화 대상:**

*   **물리 파라미터 (Physics Parameters):**
    *   **로봇 관련:**
        *   **질량 및 관성 텐서 (Mass and Inertia Tensor):** 각 링크(link)의 질량과 관성 모멘트를 무작위로 변경합니다.
        *   **관절 마찰력 및 감쇠 (Joint Friction and Damping):** 각 관절의 마찰 계수와 감쇠 계수를 변경합니다.
        *   **모터 강도 및 지연 (Motor Strength and Latency):** 액추에이터의 최대 토크/힘, 응답 지연 등을 무작위화합니다.
        *   **물리 재질 속성 (Physics Material Properties):** 로봇 발바닥이나 물체의 마찰 계수, 반발 계수 등을 변경합니다.
    *   **환경 관련:**
        *   **지면 마찰력 및 반발 계수:** 바닥의 물리적 특성을 변경합니다.
        *   **외부 힘 및 토크:** 로봇에 예기치 않은 외부 힘이나 토크를 주기적으로 또는 무작위로 가합니다 (예: 바람, 충격).
*   **시각적 요소 (Visual Aspects):**
    *   **텍스처 (Textures):** 로봇, 물체, 바닥, 벽 등의 텍스처를 무작위로 변경합니다. 다양한 색상, 패턴, 재질 느낌을 적용할 수 있습니다.
    *   **조명 (Lighting):** 조명의 위치, 방향, 색상, 강도 등을 무작위로 변경합니다. 그림자와 하이라이트의 변화를 유도합니다.
    *   **카메라 위치 및 각도 (Camera Position and Orientation):** 로봇에 부착된 카메라의 상대적인 위치나 각도를 약간씩 변경하여 관찰 값에 변화를 줍니다. (에이전트가 사용하는 카메라에 해당)
    *   **배경 및 환경 객체:** 배경의 모습이나 주변에 배치된 관련 없는 객체들의 위치, 모양, 색상 등을 변경합니다.
*   **센서 관련 (Sensor Aspects):**
    *   **센서 노이즈:** 카메라 이미지 노이즈(가우시안, 소금-후추 등), 라이다 측정값 오류, IMU 센서 바이어스 및 노이즈 등을 추가하고 그 강도를 무작위화합니다.
    *   **센서 지연 (Sensor Latency):** 센서 데이터가 에이전트에게 전달되기까지의 시간을 무작위로 지연시킵니다.
*   **작업 관련 파라미터 (Task-related Parameters):**
    *   **초기 상태 무작위화:** 로봇의 초기 자세(joint positions), 초기 속도, 목표물의 초기 위치 등을 무작위로 설정합니다.
    *   **목표 무작위화:** 매 에피소드마다 로봇이 도달해야 할 목표 지점이나 달성해야 할 목표 값(예: 특정 속도, 특정 힘)을 변경합니다.

**구현 기법:**

1.  **Orbit의 이벤트 콜백 활용:**
    *   Isaac Lab (Orbit)은 다양한 시뮬레이션 이벤트(예: `on_simulation_play`, `on_physics_step`, `on_timeline_event`, 그리고 환경의 `reset_idx` 함수 내부)에 대한 콜백 함수를 등록할 수 있는 기능을 제공합니다.
    *   이러한 콜백 내에서 무작위화 로직을 실행합니다. 예를 들어, 매 에피소드 시작 시(`reset_idx` 또는 특정 리셋 이벤트 콜백) 물리 파라미터나 시각적 요소를 변경할 수 있습니다.
    *   **예시 (`reset_idx` 내에서 물리 파라미터 변경):**
        ```python
        # Isaac Lab 환경 클래스 내 (예: AntEnv)
        # def reset_idx(self, env_ids: torch.Tensor):
        #     super().reset_idx(env_ids) # 부모 클래스의 리셋 호출
            
        #     for env_id in env_ids:
        #         # 예시: 특정 링크의 질량 무작위화
        #         link_prim = self.scene.get_object("my_robot").prim # 로봇 프림 가져오기 (이름은 실제와 다를 수 있음)
        #         # robot_articulation = self.scene.robots["robot_name"] # 또는 매니저 사용
        #         # body_names = robot_articulation.body_names
        #         # target_link = robot_articulation.body(body_names[0]) # 예시: 첫 번째 바디(링크)
                
        #         # 이 부분은 USD API나 Isaac Sim Core API를 사용하여 실제 프림의 물리 속성에 접근해야 함
        #         # 예: mass_api = UsdPhysics.MassAPI.Get(self.stage, target_link_path)
        #         # if mass_api:
        #         #    original_mass = mass_api.GetMassAttr().Get()
        #         #    random_mass_factor = np.random.uniform(0.8, 1.2) # 80% ~ 120%
        #         #    mass_api.GetMassAttr().Set(original_mass * random_mass_factor)
                
        #         # 예시: 지면 마찰력 무작위화
        #         # ground_prim = self.scene.get_object("ground_plane").prim
        #         # physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(ground_prim)
        #         # static_friction = np.random.uniform(0.4, 1.0)
        #         # dynamic_friction = np.random.uniform(0.3, 0.8)
        #         # physx_material_api.CreateStaticFrictionAttr().Set(static_friction)
        #         # physx_material_api.CreateDynamicFrictionAttr().Set(dynamic_friction)
        #         pass # 실제 구현은 Isaac Sim API 사용 필요
        ```
        **참고:** 위 코드는 개념적인 예시입니다. 실제 물리 속성 변경은 USD API (`pxr.UsdPhysics`, `pxr.PhysxSchema`) 또는 Isaac Sim Core의 물리 관련 유틸리티를 사용하여 수행해야 합니다. `self.scene` 객체나 각 매니저(`RobotManager`, `AssetManager` 등)를 통해 프림 및 해당 속성에 접근할 수 있습니다.

2.  **매니저(Manager) 및 설정 파일 활용:**
    *   Isaac Lab의 환경은 다양한 매니저(예: `SceneManager`, `RobotManager`, `TaskManager`, `ObservationManager`, `RandomizationManager`)를 사용하여 구성됩니다.
    *   `RandomizationManager` (또는 유사한 이름의 커스텀 매니저)를 사용하여 특정 무작위화 작업을 그룹화하고 관리할 수 있습니다.
    *   환경 설정 파일(TOML 또는 Python)에서 각 무작위화 항목의 범위(최소/최대값), 주기, 적용 대상 등을 정의하고, 매니저가 이를 읽어 실행하도록 구성할 수 있습니다.
    *   예를 들어, 텍스처 무작위화의 경우, 사용할 텍스처 파일 목록과 적용할 프림 경로를 설정 파일에 정의하고, 매 에피소드 시작 시 무작위로 텍스처를 선택하여 적용하는 로직을 구현합니다.

3.  **USD API 및 Isaac Sim Core API 직접 사용:**
    *   Python 스크립트 내에서 `pxr.Usd`, `pxr.Gf`, `omni.isaac.core.utils` 등의 API를 직접 사용하여 프림의 속성(예: 물리 재질, 시각적 머티리얼, 조명 파라미터)을 동적으로 변경합니다.
    *   **텍스처 변경 예시 (개념):**
        ```python
        # from pxr import UsdShade, Sdf
        # target_prim_path = "/World/MyObject"
        # new_texture_path = "/path/to/random/texture.png" # 또는 Nucleus 경로
        # prim = self.stage.GetPrimAtPath(target_prim_path)
        # material_path = ... # 프림에 연결된 머티리얼 경로 가져오기
        # material = UsdShade.Material.Get(self.stage, material_path)
        # if material:
        #    shader_prim = material.GetSurfaceOutput().Get셰이더Prim() # 실제 API는 다를 수 있음
        #    if shader_prim:
        #        diffuse_texture_input = shader_prim.GetInput("diffuse_texture")
        #        if diffuse_texture_input:
        #            diffuse_texture_input.Set(Sdf.AssetPath(new_texture_path))
        ```
    *   **조명 변경 예시 (개념):**
        ```python
        # from pxr import UsdLux, Gf
        # light_prim_path = "/World/MyDistantLight"
        # light_prim = self.stage.GetPrimAtPath(light_prim_path)
        # if light_prim:
        #    light_api = UsdLux.LightAPI(light_prim) # 실제 API는 다를 수 있음 (예: DistantLightAPI)
        #    if light_api:
        #        # 색상 변경
        #        new_color = Gf.Vec3f(np.random.rand(), np.random.rand(), np.random.rand())
        #        light_api.GetColorAttr().Set(new_color)
        #        # 강도 변경
        #        new_intensity = np.random.uniform(500, 2000)
        #        light_api.GetIntensityAttr().Set(new_intensity)
        ```

## 3. 설정 및 예제

**설정 옵션:**

도메인 무작위화를 적용할 때는 일반적으로 다음과 같은 사항을 설정하거나 코드 내에서 관리합니다.

*   **무작위화 대상 파라미터:** 어떤 물리 속성, 시각적 요소, 센서 파라미터를 변경할지 목록을 정의합니다.
*   **분포 및 범위:** 각 파라미터를 어떤 확률 분포(예: 균등 분포, 정규 분포)에서 샘플링할지, 그리고 샘플링할 값의 최소/최대 범위 또는 평균/표준편차를 지정합니다.
*   **적용 주기 (Frequency):**
    *   **에피소드마다 (Per-episode):** 매 학습 에피소드가 시작될 때마다 파라미터를 변경합니다. 가장 일반적인 방식입니다.
    *   **스텝마다 (Per-step):** 매 시뮬레이션 스텝(또는 몇 스텝마다) 파라미터를 변경합니다. 더 동적인 환경을 만들 수 있지만, 시뮬레이션 안정성과 학습 속도에 영향을 줄 수 있습니다.
    *   **특정 조건 만족 시:** 특정 이벤트가 발생하거나 작업의 특정 단계에 도달했을 때 파라미터를 변경합니다.
*   **상관 관계 (Correlations):** 여러 파라미터를 동시에 변경할 때, 이들 간의 상관 관계를 고려하여 함께 변경할지 여부를 결정할 수 있습니다. (고급 기법)

**예제 시나리오: 로봇 팔 물체 집기 작업**

*   **목표:** 다양한 조건에서 물체를 안정적으로 집는 정책 학습.
*   **무작위화 대상:**
    *   **물리:**
        *   로봇 팔 각 관절의 마찰력 (범위: 실제 값의 +/- 20%)
        *   그리퍼 손가락의 마찰 계수 (범위: 0.5 ~ 1.2)
        *   집을 물체의 질량 (범위: 목표 질량의 +/- 30%)
        *   물체의 물리 재질 (마찰력, 반발 계수)
        *   (선택 사항) 로봇에 가해지는 약한 무작위 외부 힘
    *   **시각:**
        *   물체의 색상 또는 텍스처 (미리 정의된 여러 텍스처 중 무작위 선택)
        *   테이블 표면의 텍스처
        *   배경 텍스처 또는 색상
        *   조명의 위치 및 강도 (여러 조명 프리셋 중 선택 또는 범위 내 무작위화)
        *   카메라의 extrinsic 파라미터 (위치/방향 약간 변경)
    *   **작업:**
        *   물체의 초기 위치 및 방향 (테이블 위 특정 영역 내에서 무작위)
        *   로봇 팔의 초기 관절 각도 (약간의 노이즈 추가)
*   **구현:**
    *   환경의 `reset_idx` 함수 또는 관련 이벤트 콜백 내에서 위 파라미터들을 `np.random.uniform`, `np.random.normal` 등을 사용하여 무작위로 샘플링하고, USD 및 Isaac Sim API를 통해 해당 프림 속성에 적용합니다.
    *   텍스처 변경의 경우, 미리 로드된 여러 머티리얼 프림을 준비해두고, 매번 다른 머티리얼을 물체에 바인딩하는 방식을 사용할 수 있습니다.

**팁:**

*   **점진적 적용:** 처음부터 모든 것을 무작위화하기보다는, 중요한 파라미터부터 시작하여 점진적으로 무작위화의 범위와 대상을 늘려나가는 것이 좋습니다.
*   **범위 설정의 중요성:** 무작위화 범위가 너무 좁으면 효과가 미미하고, 너무 넓으면 학습이 불안정해지거나 불가능해질 수 있습니다. 실제 환경에서 관찰될 수 있는 변화의 범위를 고려하여 적절한 범위를 설정해야 합니다.
*   **디버깅 및 모니터링:** 도메인 무작위화가 적용된 환경에서 에이전트가 어떻게 행동하는지, 특정 파라미터 변화가 학습에 어떤 영향을 미치는지 주의 깊게 관찰하고 디버깅하는 것이 중요합니다. (예: 특정 무작위화 조합에서만 실패하는 경우)
*   **체계적인 실험:** 어떤 무작위화 요소가 Sim-to-Real 성능에 가장 큰 영향을 미치는지 체계적인 실험을 통해 파악하고, 중요한 요소에 집중하는 것이 효율적일 수 있습니다.

도메인 무작위화는 Isaac Lab을 사용하여 실제 환경에 적용 가능한 강인한 로봇 RL 정책을 개발하는 데 있어 핵심적인 역할을 합니다. Isaac Lab의 유연한 Python API와 Omniverse의 강력한 USD 기반 씬 관리 기능을 활용하여 효과적인 도메인 무작위화 전략을 수립하고 구현해 보시기 바랍니다.
