# Isaac Lab 설치 가이드 (Isaac Lab Installation Guide)

Isaac Lab을 사용하여 로봇 강화학습 연구를 시작하려면 먼저 시스템에 올바르게 설치해야 합니다. 이 가이드에서는 Isaac Lab 설치에 필요한 사전 요구 사항부터 단계별 설치 과정, 그리고 설치 확인 방법까지 상세히 안내합니다.

## 1. 사전 요구 사항 (Prerequisites)

Isaac Lab을 설치하고 실행하기 전에 다음 사항들이 준비되어 있어야 합니다.

*   **NVIDIA Isaac Sim 설치 및 실행 가능 상태:**
    *   Isaac Lab은 Isaac Sim 위에서 실행됩니다. 따라서 [Isaac Sim 설치 가이드](./../isaac_sim/beginner_tutorials/2_installation_guide.md)를 참조하여 Isaac Sim (권장 버전: 최신 릴리스 또는 Isaac Lab과 호환되는 특정 버전)이 먼저 올바르게 설치되고 실행될 수 있어야 합니다.
    *   Omniverse Launcher를 통해 Isaac Sim을 실행하고 기본적인 예제(예: `Simple_Scene.usd`)가 정상적으로 로드되는지 확인하십시오.
*   **운영 체제:**
    *   Ubuntu 20.04 LTS 또는 22.04 LTS (권장)
*   **NVIDIA 드라이버:**
    *   Isaac Sim과 호환되는 최신 NVIDIA 드라이버 (Studio 또는 Game Ready). Isaac Sim 설치 가이드에서 드라이버 요구 사항을 확인하십시오.
*   **Conda (Miniconda 또는 Anaconda):**
    *   Isaac Lab은 Python 환경 관리를 위해 Conda를 사용하는 것을 강력히 권장합니다. Conda는 패키지 및 의존성 관리를 용이하게 하여 충돌을 방지합니다.
    *   **Miniconda 설치 (권장):**
        1.  [Miniconda 웹사이트](https://docs.conda.io/en/latest/miniconda.html) 방문.
        2.  Linux용 Python 3.8 또는 3.10 버전 Miniconda 설치 스크립트 다운로드 (Isaac Lab은 특정 Python 버전을 요구할 수 있으므로 공식 문서 확인).
        3.  터미널에서 스크립트 실행: `bash Miniconda3-latest-Linux-x86_64.sh`
        4.  설치 지침에 따라 진행 (라이선스 동의, 설치 경로 지정 등).
        5.  설치 후 `conda init`을 실행하라는 메시지가 나오면 따르거나, 터미널을 재시작하여 `conda` 명령어가 인식되는지 확인합니다.
    *   **Anaconda 설치:** 이미 Anaconda가 설치되어 있다면 그것을 사용해도 됩니다.
*   **Python 버전:**
    *   Isaac Lab은 특정 Python 버전을 요구합니다 (예: Python 3.8 또는 3.10). Isaac Lab 공식 문서에서 현재 권장하는 Python 버전을 확인하십시오. Conda를 사용하면 특정 버전의 Python 환경을 쉽게 만들 수 있습니다.
*   **Git:**
    *   Isaac Lab 소스 코드를 복제(clone)하기 위해 Git이 필요합니다.
    *   설치 (Ubuntu): `sudo apt update && sudo apt install git`
*   **인터넷 연결:**
    *   Isaac Lab 저장소 복제 및 필요한 패키지 다운로드를 위해 필요합니다.

## 2. Isaac Lab 단계별 설치 방법

사전 요구 사항이 모두 충족되었다면, 다음 단계에 따라 Isaac Lab을 설치합니다.

**단계 1: Isaac Lab 저장소 복제 (Cloning the Isaac Lab Repository)**

1.  터미널을 엽니다.
2.  Isaac Lab을 설치할 디렉토리로 이동합니다. 예를 들어, 홈 디렉토리 아래에 `Developer` 폴더를 만들고 그 안으로 이동할 수 있습니다.
    ```bash
    mkdir -p ~/Developer
    cd ~/Developer
    ```
3.  Isaac Lab의 공식 GitHub 저장소를 복제합니다.
    ```bash
    git clone https://github.com/NVIDIA-Omniverse/IsaacLab.git
    ```
4.  복제가 완료되면 `IsaacLab` 디렉토리가 생성됩니다. 이 디렉토리로 이동합니다.
    ```bash
    cd IsaacLab
    ```
    이 디렉토리를 Isaac Lab의 루트 디렉토리 (`ISAACLAB_PATH`)라고 부릅니다.

**단계 2: Conda 환경 생성 및 활성화 (Creating and Activating Conda Environment)**

Isaac Lab은 격리된 Python 환경에서 실행하는 것이 좋습니다.

1.  Isaac Lab 루트 디렉토리(`IsaacLab`) 내에서 다음 명령어를 사용하여 Conda 환경을 생성합니다. Isaac Lab은 필요한 Python 버전 (예: 3.10)과 환경 이름을 `conda-env.yaml` 또는 유사한 파일에 정의해 두거나, 설치 스크립트 내에서 지정할 수 있습니다. 공식 문서에서 권장하는 Python 버전을 확인하고, 만약 특정 버전이 명시되어 있다면 해당 버전을 사용해야 합니다.
    ```bash
    # 예시: Python 3.10을 사용하는 'isaaclab' 이름의 환경 생성
    conda create -n isaaclab python=3.10 
    ```
    또는 Isaac Lab에서 제공하는 환경 설정 파일을 사용할 수도 있습니다. (예: `conda env create -f environment.yml`)
2.  생성된 Conda 환경을 활성화합니다.
    ```bash
    conda activate isaaclab
    ```
    이제 터미널 프롬프트 앞에 `(isaaclab)`와 같이 현재 활성화된 환경 이름이 표시됩니다.

**단계 3: Isaac Lab 설치 스크립트 실행 (Running the Isaac Lab Installation Script)**

Isaac Lab은 필요한 모든 종속 패키지를 설치하고 환경을 설정하는 편리한 스크립트를 제공합니다.

1.  Conda 환경이 활성화된 상태에서 Isaac Lab 루트 디렉토리(`IsaacLab`)인지 확인합니다.
2.  설치 스크립트를 실행합니다. 이 스크립트는 일반적으로 `isaaclab.sh` 또는 `setup.sh`와 같은 이름으로 제공됩니다.
    ```bash
    ./isaaclab.sh # 또는 해당 스크립트 이름
    ```
    스크립트는 다음 작업들을 수행할 수 있습니다:
    *   필수 Python 패키지 설치 (예: PyTorch, NumPy, skrl 등)
    *   Isaac Sim과의 연결 설정
    *   필요한 환경 변수 설정 또는 확인 안내
    *   미리 컴파일된 확장 기능 빌드 (필요한 경우)

    설치 과정 중 사용자 입력이 필요할 수 있습니다 (예: 라이선스 동의, 경로 확인 등). 화면의 지시를 주의 깊게 따르십시오.

**단계 4: 환경 변수 설정 (Setting Environment Variables - 필요한 경우)**

Isaac Lab 또는 Isaac Sim이 특정 환경 변수를 요구할 수 있습니다. 설치 스크립트가 자동으로 `.bashrc`나 유사한 쉘 설정 파일에 추가해 주기도 하지만, 수동 설정이 필요할 수도 있습니다.

*   예를 들어, `ISAAC_PATH` (Isaac Sim 설치 경로)나 `NUCLEUS_SERVER` (Nucleus 서버 주소) 등이 필요할 수 있습니다.
*   설치 과정이나 Isaac Lab 공식 문서에서 필요한 환경 변수와 설정 방법을 확인하십시오.
*   환경 변수를 `.bashrc` 파일에 추가한 경우, 변경 사항을 적용하기 위해 `source ~/.bashrc` 명령을 실행하거나 새 터미널을 여십시오.

## 3. 설치 확인 방법 (Verifying the Installation)

설치가 성공적으로 완료되었는지 확인하기 위해 Isaac Lab에서 제공하는 예제 환경을 실행해 봅니다.

1.  **Conda 환경 활성화:** 아직 활성화되지 않았다면 Isaac Lab Conda 환경을 활성화합니다.
    ```bash
    conda activate isaaclab
    ```
2.  **Isaac Lab 루트 디렉토리로 이동:**
    ```bash
    cd /path/to/your/IsaacLab # IsaacLab을 복제한 경로로 변경
    ```
3.  **예제 환경 실행 스크립트 실행:** Isaac Lab은 일반적으로 `run_env.sh` 또는 유사한 이름의 스크립트를 통해 예제 환경을 실행합니다. 실행할 수 있는 환경 목록이나 특정 환경을 지정하는 방법은 공식 문서를 참조하십시오.
    ```bash
    # 예시: Cartpole 환경 실행 (스크립트와 인자는 실제와 다를 수 있음)
    ./orbit.sh -p source/standalone/environments/cartpole.py 
    # 또는 python source/standalone/environments/cartpole.py
    # 또는 python -m omni.isaac.lab.example_envs.cartpole --headless (최신 Isaac Lab 방식)
    ```
    최신 Isaac Lab (Orbit 기반)에서는 다음과 같은 명령 형식을 사용할 수 있습니다:
    ```bash
    # 예시: Ant 환경 실행 (headless 모드)
    python -m omni.isaac.lab.examples.rl_games.train --task Ant --headless
    # 또는 특정 환경 스크립트 직접 실행
    python workflows/train.py --task Isaac-Ant-v0
    ```
    **주의:** 정확한 실행 명령어는 Isaac Lab 버전 및 구조에 따라 다를 수 있습니다. 반드시 공식 문서나 저장소의 `README.md` 파일에서 최신 실행 방법을 확인하십시오.
4.  **Isaac Sim 실행 및 환경 로드 확인:**
    *   `--headless` 옵션 없이 실행하면 Isaac Sim 윈도우가 나타나고 예제 환경(예: 로봇, 물체 등)이 로드되어 시뮬레이션이 시작되는 것을 볼 수 있어야 합니다.
    *   `--headless` 모드로 실행하면 그래픽 인터페이스 없이 백그라운드에서 시뮬레이션이 실행되며, 터미널에 로그가 출력됩니다.
    *   에러 메시지 없이 시뮬레이션이 시작되고, 로봇이 움직이거나 학습이 진행되는 로그가 나타나면 설치가 성공적으로 완료된 것입니다.

## 4. 일반적인 설치 문제 해결 (Troubleshooting Common Installation Issues)

*   **`command not found: conda`:**
    *   Conda가 올바르게 설치되지 않았거나 PATH에 추가되지 않았습니다. Miniconda 설치를 다시 확인하고, `conda init`을 실행했는지, 터미널을 재시작했는지 확인합니다.
*   **Python 버전 불일치:**
    *   Isaac Lab이 요구하는 특정 Python 버전과 다른 버전이 Conda 환경에 설치된 경우 발생할 수 있습니다. Conda 환경 생성 시 Python 버전을 명시적으로 지정하십시오.
*   **PyTorch 설치 오류 (CUDA 관련):**
    *   PyTorch가 시스템의 CUDA 버전과 호환되지 않게 설치된 경우 발생합니다. Isaac Lab 설치 스크립트가 적절한 PyTorch 버전을 설치하도록 하지만, 문제가 발생하면 PyTorch 공식 웹사이트에서 시스템 환경(CUDA 버전, OS)에 맞는 설치 명령어를 확인하여 수동으로 재설치해 볼 수 있습니다.
    *   `nvidia-smi` 명령어로 현재 설치된 NVIDIA 드라이버와 CUDA 버전을 확인합니다.
*   **Isaac Sim 경로 문제:**
    *   Isaac Lab이 Isaac Sim 설치 경로를 찾지 못하는 경우 발생합니다. `ISAAC_PATH`와 같은 환경 변수가 올바르게 설정되었는지 확인하거나, Isaac Lab 설정 파일에서 Isaac Sim 경로를 지정해야 할 수 있습니다.
*   **Nucleus 연결 문제:**
    *   Isaac Sim이 Nucleus 서버에 연결할 수 없는 경우 발생합니다. Omniverse Launcher에서 Nucleus 서비스가 실행 중인지, 네트워크 설정에 문제가 없는지 확인합니다.
*   **`ImportError` 또는 `ModuleNotFoundError`:**
    *   필요한 Python 패키지가 설치되지 않았거나, 다른 버전과의 충돌로 인해 발생할 수 있습니다. Conda 환경이 올바르게 활성화되었는지 확인하고, Isaac Lab 설치 스크립트를 다시 실행하거나 필요한 패키지를 수동으로 설치해 봅니다.
*   **확장 기능(Extension) 빌드 또는 로드 실패:**
    *   Isaac Sim 또는 Isaac Lab의 C++/CUDA 확장 기능을 빌드하거나 로드하는 데 실패하는 경우입니다. 컴파일러(gcc/g++), CUDA Toolkit 버전 등이 호환되는지 확인해야 할 수 있습니다. Isaac Lab 공식 문서에서 빌드 요구 사항을 확인하십시오.

문제가 해결되지 않으면 Isaac Lab GitHub 저장소의 "Issues" 섹션이나 NVIDIA 개발자 포럼에서 유사한 문제를 검색하거나 질문을 올려 도움을 받을 수 있습니다.

다음 섹션에서는 Isaac Lab의 핵심 개념과 아키텍처에 대해 자세히 알아보겠습니다.
