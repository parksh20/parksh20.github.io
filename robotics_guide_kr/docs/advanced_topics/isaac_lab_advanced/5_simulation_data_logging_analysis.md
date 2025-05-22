# Isaac Lab 고급 토픽: 시뮬레이션 데이터 기록 및 분석

강화학습(RL) 연구 및 로봇 애플리케이션 개발에서 학습 과정과 시뮬레이션 결과를 체계적으로 기록하고 분석하는 것은 매우 중요합니다. 이를 통해 에이전트의 학습 진행 상황을 모니터링하고, 하이퍼파라미터 튜닝의 근거를 마련하며, 문제점을 진단하고, 연구 결과를 공유하거나 재현할 수 있습니다.

Isaac Lab은 학습 과정 중 다양한 데이터를 로깅할 수 있는 기능을 제공하며, 특히 TensorBoard와 같은 외부 도구와 연동하여 이러한 데이터를 효과적으로 시각화하고 분석할 수 있도록 지원합니다.

이 섹션에서는 Isaac Lab에서 학습 및 시뮬레이션 관련 데이터를 로깅하는 방법과, TensorBoard 및 기타 도구를 사용하여 수집된 데이터를 분석하고 인사이트를 얻는 방법에 대해 설명합니다.

## 1. Isaac Lab에서 학습 및 시뮬레이션 데이터 로깅

Isaac Lab은 주로 RL 라이브러리(예: skrl)의 로깅 기능과 연동되거나, 자체적인 로깅 유틸리티를 통해 학습 관련 지표 및 시뮬레이션 상태 정보를 기록합니다.

**주요 로깅 대상 데이터:**

*   **학습 관련 지표 (Training Metrics):**
    *   **에피소드 보상 (Episodic Reward):** 각 학습 에피소드에서 에이전트가 받은 총 보상, 평균 보상, 최대/최소 보상 등. 학습의 가장 기본적인 성공 척도입니다.
    *   **손실 함수 값 (Loss Values):** 정책망(actor) 손실, 가치망(critic) 손실, 엔트로피 손실(entropy loss) 등 RL 알고리즘 내부의 손실 함수 값 변화. 학습 안정성과 수렴 여부를 판단하는 데 사용됩니다.
    *   **학습률 (Learning Rate):** 옵티마이저의 학습률 변화 (스케줄링 사용 시).
    *   **탐색 파라미터 (Exploration Parameters):** 엔트로피 계수, 노이즈 표준편차 등 탐색 관련 파라미터의 변화.
    *   **에피소드 길이 (Episode Length):** 각 에피소드가 종료될 때까지 소요된 스텝 수.
    *   **작업 성공률 (Success Rate):** 특정 기준을 만족하여 작업에 성공한 에피소드의 비율 (작업에 따라 정의).
*   **시뮬레이션 상태 정보 (Simulation State Information):**
    *   **로봇 관절 상태:** 특정 관절의 각도, 속도, 토크 등.
    *   **로봇/물체 포즈:** 로봇 베이스 링크 또는 특정 물체의 월드 좌표계 기준 위치 및 방향.
    *   **센서 데이터 (선택적):** 특정 시점의 카메라 이미지, 라이다 스캔 데이터 등을 저장할 수 있으나, 용량이 매우 커질 수 있으므로 주의해야 합니다. (주로 디버깅이나 특정 분석 목적)
    *   **충돌 정보:** 로봇과 환경 또는 로봇 간 충돌 발생 여부 및 관련 정보.
*   **환경 파라미터 (Environment Parameters):**
    *   도메인 무작위화나 커리큘럼 학습이 적용된 경우, 현재 환경의 물리/시각 파라미터 값, 현재 난이도 수준 등을 로깅하여 학습 과정과의 연관성을 분석할 수 있습니다.

**로깅 방법 및 도구:**

1.  **RL 라이브러리의 로깅 기능 활용 (예: skrl):**
    *   skrl과 같은 대부분의 RL 라이브러리는 학습 중 주요 지표(보상, 손실 등)를 자동으로 로깅하고, 이를 TensorBoard 형식으로 저장하는 기능을 내장하고 있습니다.
    *   에이전트 설정(`cfg`)이나 트레이너(trainer) 설정에서 로깅 관련 옵션(예: 로깅 디렉토리, 로깅 주기)을 지정할 수 있습니다.
    *   **예시 (skrl 트레이너 설정 - 개념):**
        ```python
        # from skrl.trainers.torch import SequentialTrainer
        
        # cfg_trainer = {
        #     "timesteps": 100000, # 총 학습 타임스텝
        #     "progress_interval": 1000, # 진행 상황 로깅 주기 (타임스텝 단위)
        #     "experiment": {
        #         "directory": "runs/my_experiment",  # 로깅 데이터 저장 경로
        #         "experiment_name": "ant_ppo",       # 실험 이름
        #         "write_interval": 500,              # TensorBoard 기록 주기 (타임스텝 단위)
        #         "checkpoint_interval": 5000,        # 모델 체크포인트 저장 주기
        #         "store_separately": False,
        #     }
        # }
        # trainer = SequentialTrainer(env=env, agents=agent, cfg=cfg_trainer)
        # trainer.train() # 학습 시작 (이 과정에서 자동으로 로깅 수행)
        ```
    *   학습이 진행되면 `cfg_trainer["experiment"]["directory"]` 아래에 TensorBoard 로그 파일(이벤트 파일)과 모델 체크포인트가 저장됩니다.

2.  **Isaac Lab (Orbit)의 내장 로깅 유틸리티:**
    *   Orbit 프레임워크 자체에도 특정 데이터를 로깅하거나 화면에 출력하는 유틸리티가 포함될 수 있습니다.
    *   환경 클래스 내에서 `print()` 또는 Python의 `logging` 모듈을 사용하여 특정 이벤트나 상태 값을 콘솔에 출력하거나 파일로 저장할 수 있습니다.
    *   커스텀 데이터를 TensorBoard에 직접 기록하고 싶다면, Isaac Lab 환경 스크립트 내에서 `torch.utils.tensorboard.SummaryWriter`를 직접 사용하여 구현할 수 있습니다.

3.  **커스텀 TensorBoard 로깅 구현:**
    *   RL 라이브러리가 기본적으로 제공하지 않는 커스텀 데이터(예: 특정 작업 성공 지표, 환경의 내부 상태 값)를 TensorBoard에 로깅하고 싶을 때 사용합니다.
    *   **구현 단계:**
        1.  `torch.utils.tensorboard.SummaryWriter` 객체를 생성합니다.
            ```python
            from torch.utils.tensorboard import SummaryWriter
            # 로그 저장 경로 지정
            log_dir = "runs/my_custom_logs/" + datetime.now().strftime("%Y%m%d-%H%M%S")
            writer = SummaryWriter(log_dir)
            ```
        2.  학습 루프 내에서 원하는 데이터를 `writer.add_scalar()`, `writer.add_histogram()`, `writer.add_image()` 등의 메서드를 사용하여 기록합니다.
            ```python
            # 예시: 커스텀 성공률 로깅
            # global_step = trainer.total_timesteps # 또는 직접 관리하는 스텝 카운터
            # success_rate = calculate_my_success_rate(...) 
            # writer.add_scalar("Custom/SuccessRate", success_rate, global_step)
            
            # 예시: 특정 관절 각도 로깅
            # joint_angle = env.get_joint_angle("joint_1") 
            # writer.add_scalar("RobotState/Joint1_Angle", joint_angle, global_step)
            ```
        3.  학습 종료 후 `writer.close()`를 호출합니다.

4.  **시뮬레이션 데이터셋 저장 (고급):**
    *   RL 학습 외의 목적으로, 시뮬레이션에서 생성된 대규모 데이터셋(예: 합성 이미지 데이터와 라벨, 로봇의 이동 경로와 센서 값)을 저장해야 할 수 있습니다.
    *   이 경우, HDF5, Zarr, CSV, 또는 커스텀 파일 형식을 사용하여 데이터를 효율적으로 저장하고 관리하는 파이프라인을 구축해야 합니다.
    *   데이터 수집 주기, 저장 형식, 메타데이터 관리 등을 신중하게 설계해야 합니다.
    *   Isaac Replicator (합성 데이터 생성 도구)와 같은 기능을 활용할 수도 있습니다.

## 2. TensorBoard 및 기타 도구를 사용한 분석

로깅된 데이터는 학습 과정의 이해, 모델 성능 평가, 문제 해결에 매우 중요한 역할을 합니다.

**TensorBoard 활용:**

TensorBoard는 TensorFlow와 함께 제공되는 시각화 도구이지만, PyTorch를 포함한 다른 프레임워크에서도 널리 사용됩니다. (skrl, `torch.utils.tensorboard.SummaryWriter` 모두 TensorBoard 형식으로 로그 생성)

1.  **TensorBoard 실행:**
    *   터미널에서 로깅 디렉토리의 **상위 폴더**를 지정하여 TensorBoard를 실행합니다.
        ```bash
        # 예: 로그가 runs/my_experiment 폴더에 있다면
        tensorboard --logdir runs 
        # 또는 특정 실험만 보고 싶다면
        # tensorboard --logdir runs/my_experiment
        ```
    *   웹 브라우저를 열고 화면에 표시된 주소(보통 `http://localhost:6006`)로 접속합니다.

2.  **주요 기능 및 분석 방법:**
    *   **SCALARS 탭:**
        *   시간(학습 스텝 또는 에피소드)에 따른 스칼라 값(보상, 손실, 성공률, 에피소드 길이 등)의 변화를 그래프로 보여줍니다.
        *   여러 실험 실행 결과를 오버레이하여 비교할 수 있습니다 (예: 다른 하이퍼파라미터 설정 간의 성능 비교).
        *   그래프의 스무딩(Smoothing) 기능을 사용하여 노이즈가 많은 곡선을 부드럽게 만들어 추세를 파악하기 쉽게 할 수 있습니다.
        *   **분석 포인트:**
            *   보상 곡선이 꾸준히 상승하는가? (학습이 잘 진행되고 있는가?)
            *   손실 곡선이 안정적으로 수렴하는가? (발산하거나 진동이 심하지 않은가?)
            *   성공률이나 에피소드 길이가 개선되는가?
            *   다른 하이퍼파라미터 설정이나 알고리즘 변경이 성능에 어떤 영향을 미치는가?
    *   **HISTOGRAMS 및 DISTRIBUTIONS 탭:**
        *   가중치, 편향, 활성화 값, 그래디언트 등 텐서 값들의 분포가 시간에 따라 어떻게 변하는지 시각화합니다.
        *   신경망의 특정 계층에서 값이 너무 커지거나 작아지는 문제, 그래디언트 소실/폭주 문제 등을 진단하는 데 도움이 될 수 있습니다.
    *   **GRAPHS 탭:**
        *   신경망 모델의 계산 그래프 구조를 시각화합니다. (모델이 올바르게 구성되었는지 확인)
    *   **IMAGES 탭:**
        *   `writer.add_image()`로 기록한 이미지 데이터(예: 환경 관찰 값, 에이전트의 내부 표현)를 볼 수 있습니다.
    *   **TEXT 탭:**
        *   `writer.add_text()`로 기록한 텍스트 정보를 볼 수 있습니다. (예: 하이퍼파라미터 설정, 실험 노트)
    *   **HPARAMS 탭 (하이퍼파라미터 튜닝):**
        *   여러 하이퍼파라미터 조합으로 실행한 실험 결과를 표 형태로 비교하고, 각 하이퍼파라미터가 성능 지표에 미치는 영향을 분석할 수 있습니다.

**기타 분석 도구 및 방법:**

*   **Python 스크립트 (NumPy, Pandas, Matplotlib):**
    *   TensorBoard 이벤트 파일을 직접 읽거나(예: `tensorflow.python.summary.summary_iterator` 사용), RL 라이브러리가 제공하는 로그 파일(CSV 등)을 Pandas DataFrame으로 로드하여 더 세밀하고 커스터마이징된 분석 및 시각화를 수행할 수 있습니다.
    *   여러 실험 결과에 대한 통계적 유의성 검증, 특정 조건에서의 성능 비교, 복잡한 그래프 생성 등이 가능합니다.
*   **Jupyter Notebook / Google Colab:**
    *   데이터 분석 및 시각화 코드를 인터랙티브하게 실행하고 결과를 문서화하는 데 매우 유용합니다.
*   **데이터베이스 및 데이터 웨어하우스:**
    *   매우 많은 수의 실험을 장기간 관리하거나 팀 단위로 공유해야 하는 경우, 실험 결과를 데이터베이스(예: SQLite, PostgreSQL)나 데이터 웨어하우스에 저장하고 SQL 등을 사용하여 분석하는 것을 고려할 수 있습니다.
*   **비디오 로깅 및 분석:**
    *   학습된 에이전트가 특정 작업을 수행하는 모습을 비디오로 녹화하여 정성적인 분석을 수행합니다. 에이전트의 성공/실패 사례, 예기치 않은 행동 패턴 등을 파악하는 데 도움이 됩니다.
    *   Isaac Lab 환경은 화면 녹화 기능을 제공하거나, `omni.kit. fenêtre_recorder`와 같은 도구를 사용하여 비디오를 생성할 수 있습니다.
*   **체크포인트 분석:**
    *   학습 과정 중 저장된 모델 체크포인트를 로드하여 특정 시점의 에이전트 성능을 다시 평가하거나, 다른 환경에서 테스트해 볼 수 있습니다.

**효과적인 데이터 분석을 위한 팁:**

*   **일관된 로깅:** 여러 실험 결과를 비교하려면 로깅되는 데이터의 종류와 형식이 일관되어야 합니다.
*   **충분한 정보 기록:** 실험을 재현하거나 문제를 진단하는 데 필요한 모든 정보(하이퍼파라미터, 사용된 코드 버전, 환경 설정 등)를 함께 기록하는 것이 좋습니다. (Git 커밋 해시, 설정 파일 복사 등)
*   **가설 기반 분석:** 데이터를 분석하기 전에 어떤 질문에 답을 얻고 싶은지, 어떤 가설을 검증하고 싶은지 명확히 하는 것이 중요합니다.
*   **시각화의 힘:** 다양한 그래프와 시각화 기법을 활용하여 데이터로부터 패턴과 인사이트를 효과적으로 발견할 수 있습니다.

체계적인 데이터 로깅과 분석은 RL 연구 개발의 효율성을 크게 높이고, 더 나은 의사결정을 내리는 데 필수적인 과정입니다. Isaac Lab과 TensorBoard 등의 도구를 적극적으로 활용하여 시뮬레이션 결과를 깊이 있게 이해하고 개선해 나가시기 바랍니다.
