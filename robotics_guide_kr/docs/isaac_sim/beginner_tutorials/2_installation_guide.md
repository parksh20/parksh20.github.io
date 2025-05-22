# Isaac Sim 설치 가이드 (Isaac Sim Installation Guide)

Isaac Sim을 사용하기 위한 첫 번째 단계는 시스템 요구 사항을 확인하고 Omniverse Launcher를 통해 Isaac Sim을 설치하는 것입니다. 이 가이드에서는 단계별 설치 과정과 일반적인 문제 해결 방법을 안내합니다.

## 1. 시스템 요구 사항 (System Requirements)

Isaac Sim은 고성능 시뮬레이션을 위해 특정 하드웨어 및 소프트웨어 사양을 필요로 합니다. 설치 전에 반드시 시스템이 다음 요구 사항을 충족하는지 확인하십시오.

*   **운영 체제 (Operating System):**
    *   Ubuntu 20.04 LTS 또는 Ubuntu 22.04 LTS
    *   Windows 10 (일부 기능 제한 가능성 있음, Linux 권장)
*   **CPU:**
    *   Intel Core i7 또는 AMD Ryzen 7 이상 (최소 6코어 권장)
*   **RAM:**
    *   최소 32GB RAM (64GB 이상 권장, 특히 대규모 환경이나 여러 로봇을 시뮬레이션할 경우)
*   **GPU (NVIDIA 그래픽 카드 필수):**
    *   NVIDIA RTX 지원 GPU (예: GeForce RTX 3070 이상, Quadro RTX 5000 이상)
    *   최소 8GB VRAM (12GB VRAM 이상 권장, 레이 트레이싱 및 고해상도 렌더링을 위해 16GB+ 권장)
*   **NVIDIA 드라이버:**
    *   최신 NVIDIA Studio 드라이버 또는 Game Ready 드라이버. (Isaac Sim 릴리스 노트에서 권장하는 특정 드라이버 버전 확인 권장)
    *   Linux의 경우, `nvidia-driver-xxx` (xxx는 버전 번호) 패키지를 통해 설치.
*   **저장 공간 (Storage):**
    *   최소 100GB의 SSD 여유 공간 (Omniverse Launcher, Nucleus, Isaac Sim 및 캐시 파일 포함). NVMe SSD 권장.
*   **인터넷 연결:**
    *   Omniverse Launcher 다운로드, Isaac Sim 설치, Nucleus 서버 연결 및 에셋 다운로드를 위해 필요합니다.

**참고:** 위 사양은 최소 요구 사항이며, 더 복잡하고 큰 규모의 시뮬레이션을 원활하게 실행하려면 이보다 높은 사양의 시스템이 필요할 수 있습니다. 최신 정보는 항상 NVIDIA 공식 Isaac Sim 문서를 참조하십시오.

## 2. Omniverse Launcher 및 Isaac Sim 단계별 설치 방법

Isaac Sim은 NVIDIA Omniverse Launcher를 통해 설치 및 관리됩니다.

**단계 1: NVIDIA Omniverse Launcher 다운로드 및 설치**

1.  **NVIDIA Omniverse 웹사이트 방문:** [NVIDIA Omniverse 공식 웹사이트](https://www.nvidia.com/en-us/omniverse/)로 이동합니다.
2.  **Launcher 다운로드:** "GET OMNIVERSE" 또는 "DOWNLOAD LAUNCHER" 버튼을 클릭하여 운영 체제에 맞는 Omniverse Launcher 설치 파일을 다운로드합니다.
    *   Linux 사용자의 경우 `.AppImage` 또는 `.deb` 파일을 받을 수 있습니다.
3.  **Launcher 설치:**
    *   **Linux:**
        *   `.AppImage` 파일: 실행 권한을 부여 (`chmod +x Omniverse_Launcher-xxx.AppImage`)한 후 실행 (`./Omniverse_Launcher-xxx.AppImage`).
        *   `.deb` 파일: `sudo apt install ./omniverse-launcher-xxx.deb` 명령어로 설치.
    *   **Windows:** 다운로드한 `.exe` 파일을 실행하고 지침에 따라 설치합니다.
4.  **Launcher 실행 및 로그인:** 설치가 완료되면 Omniverse Launcher를 실행합니다. NVIDIA 개발자 계정으로 로그인해야 합니다. 계정이 없다면 화면의 안내에 따라 새 계정을 생성합니다.

**단계 2: Nucleus 설정 (로컬 또는 원격)**

Nucleus는 Omniverse 프로젝트와 에셋을 저장하고 협업하기 위한 데이터베이스 및 서버 애플리케이션입니다. 로컬에 설치하거나 기존 원격 서버에 연결할 수 있습니다.

1.  **Launcher에서 Nucleus 탭으로 이동:** Omniverse Launcher 상단 메뉴에서 "NUCLEUS" 탭을 선택합니다.
2.  **로컬 Nucleus 설치 (권장 시작 방법):**
    *   "Add Local Nucleus Service" 또는 유사한 옵션을 선택합니다.
    *   데이터 저장 경로 등 필요한 설정을 입력하고 설치를 진행합니다.
    *   관리자 계정(admin)의 비밀번호를 설정합니다.
3.  **원격 Nucleus 연결 (선택 사항):**
    *   기존에 운영 중인 원격 Nucleus 서버가 있다면 "Add Remote Nucleus Server" 또는 "Connect to Server" 옵션을 사용하여 서버 주소와 인증 정보를 입력하고 연결합니다.

**단계 3: Isaac Sim 설치**

1.  **Launcher에서 Exchange 탭으로 이동:** Omniverse Launcher 상단 메뉴에서 "EXCHANGE" 탭을 선택합니다.
2.  **Isaac Sim 검색 및 선택:** 검색창에 "Isaac Sim"을 입력하거나 앱 목록에서 Isaac Sim을 찾습니다.
3.  **Isaac Sim 설치:**
    *   Isaac Sim 항목을 클릭하면 세부 정보 페이지로 이동합니다.
    *   "INSTALL" 버튼을 클릭합니다.
    *   설치 경로를 지정하고 (기본 경로 권장) 설치를 시작합니다. 설치 과정은 인터넷 속도와 시스템 사양에 따라 시간이 다소 소요될 수 있습니다.
    *   설치가 완료되면 "LAUNCH" 버튼으로 바뀝니다.

**단계 4: Isaac Sim 최초 실행 및 설정**

1.  **Isaac Sim 실행:** Omniverse Launcher의 "LIBRARY" 탭으로 이동하여 설치된 Isaac Sim 옆의 "LAUNCH" 버튼을 클릭합니다.
2.  **캐시 및 설정:** 처음 실행 시 필요한 캐시 파일을 생성하고 일부 초기 설정을 진행할 수 있습니다.
3.  **Nucleus 연결 확인:** Isaac Sim 내에서 로컬 또는 원격 Nucleus 서버에 정상적으로 연결되었는지 확인합니다. (예: Content Browser에서 `localhost/Users/admin` 경로 확인)

이제 Isaac Sim을 사용할 준비가 되었습니다!

## 3. 일반적인 설치 문제 해결 (Troubleshooting Common Installation Issues)

*   **NVIDIA 드라이버 문제:**
    *   **증상:** Isaac Sim 실행 실패, 검은 화면, 그래픽 오류.
    *   **해결 방법:** 최신 NVIDIA Studio 드라이버 또는 권장 드라이버 버전으로 업데이트합니다. `nvidia-smi` 명령 (Linux) 또는 NVIDIA 제어판 (Windows)에서 드라이버 상태를 확인합니다.
*   **Nucleus 연결 실패:**
    *   **증상:** Nucleus 서버에 연결할 수 없음, 에셋 로드 실패.
    *   **해결 방법:**
        *   로컬 Nucleus 서비스가 실행 중인지 확인합니다. (Launcher의 Nucleus 탭 또는 시스템 서비스 확인)
        *   방화벽 설정에서 Omniverse 관련 포트(기본값: 3009, 3019 등)가 차단되지 않았는지 확인합니다.
        *   Nucleus 설치 시 설정한 사용자 이름과 비밀번호가 정확한지 확인합니다.
*   **디스크 공간 부족:**
    *   **증상:** 설치 중단, 실행 실패.
    *   **해결 방법:** Isaac Sim, Omniverse Cache, Nucleus 데이터 등을 저장할 충분한 디스크 공간을 확보합니다. (최소 100GB 권장)
*   **Launcher 로그인 문제:**
    *   **증상:** NVIDIA 계정 로그인 실패.
    *   **해결 방법:** 인터넷 연결 상태를 확인하고, NVIDIA 계정 정보가 정확한지 확인합니다. 필요한 경우 NVIDIA 웹사이트에서 비밀번호를 재설정합니다.
*   **AppImage 실행 문제 (Linux):**
    *   **증상:** `.AppImage` 파일 실행 안 됨.
    *   **해결 방법:** 파일에 실행 권한이 있는지 확인 (`chmod +x <파일명>.AppImage`). FUSE 라이브러리가 설치되어 있는지 확인 (`sudo apt install libfuse2`).
*   **"Failed to load physics backend" 오류:**
    *   **증상:** 물리 시뮬레이션 관련 오류 메시지.
    *   **해결 방법:** Isaac Sim이 올바르게 설치되었는지, GPU 드라이버가 호환되는지 확인합니다. 문제가 지속되면 Isaac Sim을 재설치해 볼 수 있습니다.

**추가 지원:**

*   **NVIDIA Omniverse 포럼:** [Omniverse Forums](https://forums.developer.nvidia.com/c/omniverse/31)
*   **Isaac Sim 공식 문서:** Omniverse Launcher 내 "Documentation" 링크 또는 NVIDIA 웹사이트에서 최신 문서를 참조하십시오.

다음 튜토리얼에서는 Isaac Sim의 사용자 인터페이스(UI)를 둘러보고 기본적인 씬(Scene)을 설정하는 방법을 알아보겠습니다.
