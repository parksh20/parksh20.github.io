# Isaac Sim 시작하기: 에셋(Asset) 가져오기

지금까지 Isaac Sim에서 기본적인 씬을 구성하고 물리 설정을 하는 방법을 배웠습니다. 하지만 대부분의 경우, 처음부터 모든 것을 만들기보다는 이미 만들어진 로봇 모델, 환경, 소품 등의 에셋(Asset)을 활용하게 됩니다. 이 튜토리얼에서는 다양한 형식의 에셋을 Isaac Sim으로 가져오는 방법을 알아봅니다.

## 1. 지원되는 에셋 형식 (Supported Asset Formats)

Isaac Sim은 다양한 3D 모델 및 로봇 기술 형식들을 지원합니다. 주요 형식은 다음과 같습니다.

*   **USD (Universal Scene Description):**
    *   NVIDIA Omniverse의 핵심 파일 형식으로, Isaac Sim에서 가장 완벽하게 지원되는 형식입니다.
    *   3D 지오메트리, 재질, 조명, 카메라, 애니메이션, 물리 설정 등 씬의 모든 요소를 포함할 수 있습니다.
    *   확장자: `.usd`, `.usda` (텍스트 기반), `.usdc` (바이너리), `.usdz` (패키지)
    *   Isaac Sim 내에서 생성되거나 다른 3D DCC(Digital Content Creation) 툴(예: Blender, Maya, 3ds Max)의 Omniverse Connector를 통해 USD로 변환된 에셋을 가져올 수 있습니다.
*   **URDF (Unified Robot Description Format):**
    *   ROS(Robot Operating System)에서 널리 사용되는 로봇 모델 기술 형식입니다.
    *   로봇의 링크(link), 관절(joint), 센서, 시각적 메시(visual mesh), 충돌 메시(collision mesh) 등을 XML 기반으로 기술합니다.
    *   확장자: `.urdf`
    *   Isaac Sim은 URDF Importer를 통해 URDF 파일을 USD 프림으로 변환하여 가져올 수 있습니다. 이 과정에서 물리 속성, 관절 구동 설정 등을 함께 가져올 수 있습니다.
*   **MJCF (MuJoCo XML Format):**
    *   MuJoCo 물리 시뮬레이터에서 사용되는 모델 기술 형식입니다.
    *   로봇 모델 및 시뮬레이션 환경을 XML 기반으로 기술합니다.
    *   확장자: `.xml`
    *   Isaac Sim은 MJCF Importer를 통해 MJCF 파일을 USD 프림으로 변환하여 가져올 수 있습니다.
*   **기타 3D 모델 형식 (FBX, OBJ, GLTF/GLB 등):**
    *   Isaac Sim은 이러한 일반적인 3D 모델 형식도 지원하지만, USD 형식으로 변환하는 과정을 거치는 것이 좋습니다.
    *   Omniverse Launcher의 "Exchange" 탭에서 "CAD Importer"나 "FBX Importer"와 같은 변환기를 설치하여 사용할 수 있거나, Isaac Sim 내에서 직접 임포트 시 USD로 변환하는 옵션이 제공될 수 있습니다.
    *   변환 과정에서 재질이나 구조가 완벽하게 유지되지 않을 수 있으므로, 가능하면 원본 DCC 툴에서 USD로 직접 익스포트하는 것이 가장 좋습니다.

## 2. 콘텐츠 브라우저를 사용한 에셋 임포트 (Using the Content Browser to Import Assets)

콘텐츠 브라우저(Content Browser)는 Isaac Sim 내에서 로컬 파일 시스템이나 Nucleus 서버에 있는 에셋을 탐색하고 씬으로 가져오는 가장 기본적인 방법입니다.

**로컬 USD 파일 가져오기:**

1.  **콘텐츠 브라우저 열기:** Isaac Sim 인터페이스에서 콘텐츠 브라우저 패널을 찾습니다. (보통 하단에 위치)
2.  **파일 시스템 탐색:**
    *   콘텐츠 브라우저 내에서 자신의 로컬 파일 시스템 경로로 이동합니다. (예: `file:/c:/Users/YourUser/Documents/MyAsset.usd` 또는 `file:///home/youruser/my_assets/robot.usd`)
    *   USD 파일이 있는 폴더로 이동합니다.
3.  **씬으로 드래그 앤 드롭:**
    *   가져오고 싶은 USD 파일(예: `my_robot.usd`)을 콘텐츠 브라우저에서 뷰포트(Viewport)나 스테이지(Stage) 패널로 드래그 앤 드롭합니다.
    *   그러면 해당 USD 파일이 현재 열려있는 씬에 프림으로 추가됩니다.
4.  **참조(Reference) 또는 페이로드(Payload)로 추가:**
    *   에셋을 드래그할 때, 단순히 복사하는 것 외에 "참조(Reference)" 또는 "페이로드(Payload)"로 가져올 수 있습니다.
        *   **참조 (Add as Reference):** 원본 USD 파일을 직접 수정하지 않고 링크 형태로 가져옵니다. 원본 파일이 업데이트되면 Isaac Sim 씬에도 반영될 수 있습니다. 이는 협업이나 에셋 관리 측면에서 매우 유용합니다.
        *   **페이로드 (Add as Payload):** 참조와 유사하지만, 씬을 열 때 해당 에셋을 로드할지 여부를 선택적으로 제어할 수 있습니다. 대규모 씬에서 성능 관리에 도움이 됩니다.
    *   일반적으로 복잡한 에셋이나 공유 에셋은 참조로 가져오는 것이 좋습니다.

**Nucleus 서버에서 에셋 가져오기:**

Nucleus 서버는 Omniverse 프로젝트와 에셋을 공유하고 협업하기 위한 공간입니다.

1.  **Nucleus 서버 연결:** Omniverse Launcher에서 Nucleus 서버(로컬 또는 원격)가 실행 중이고 연결되어 있는지 확인합니다.
2.  **콘텐츠 브라우저에서 Nucleus 탐색:**
    *   콘텐츠 브라우저에서 Nucleus 서버 경로로 이동합니다. (예: `omniverse://localhost/NVIDIA/Assets/IsaacSim/` 또는 팀에서 사용하는 원격 서버 주소)
    *   NVIDIA에서 제공하는 기본 에셋, 샘플 로봇, 환경 등을 여기서 찾을 수 있습니다.
3.  **드래그 앤 드롭:** 로컬 파일과 마찬가지로 원하는 USD 에셋을 뷰포트나 스테이지로 드래그 앤 드롭하여 씬에 추가합니다. Nucleus에 있는 에셋은 기본적으로 참조 형태로 가져옵니다.

## 3. URDF 임포터 사용하기 (Using the URDF Importer)

ROS 사용자를 위해 URDF 파일을 Isaac Sim으로 가져오는 것은 매우 중요합니다.

1.  **URDF Importer 확장 기능 활성화 (필요시):**
    *   상단 메뉴에서 **Window > Extensions**를 선택하여 확장 기능 관리자를 엽니다.
    *   검색창에 `URDF`를 입력하고 "omni.importer.urdf" 확장이 활성화(체크 표시)되어 있는지 확인합니다. 비활성화되어 있다면 활성화합니다.
2.  **URDF Importer 실행:**
    *   상단 메뉴에서 **Isaac Utils > Importer > URDF Importer**를 선택합니다.
    *   URDF Importer 설정 창이 나타납니다.
3.  **URDF 파일 선택:**
    *   **Input File:** 가져올 `.urdf` 파일의 경로를 지정합니다.
    *   **Root Link:** 로봇의 루트 링크를 지정합니다. (보통 자동으로 감지됨)
4.  **임포트 설정 조정:**
    *   **Import Physics:** 체크하면 URDF에 정의된 물리 속성(질량, 관성 등)을 가져옵니다.
    *   **Create Articulation:** 체크하면 로봇의 관절(joint)을 Isaac Sim의 Articulation(관절 시스템)으로 만듭니다. 이를 통해 관절을 제어할 수 있습니다.
    *   **Fix Base Link:** 체크하면 로봇의 베이스 링크를 월드에 고정시킵니다. (자유롭게 움직이는 로봇의 경우 해제)
    *   **Collision Meshes:** 충돌 메시 처리 방법을 선택합니다 (예: Convex Hull, Convex Decomposition).
    *   **Distance Unit:** URDF 파일에 사용된 거리 단위를 지정합니다.
    *   **Output Directory:** 변환된 USD 파일이 저장될 경로를 지정합니다. (지정하지 않으면 임시 위치에 저장)
5.  **임포트 실행:** "IMPORT" 버튼을 클릭합니다.
6.  **결과 확인:** 지정된 경로 또는 임시 경로에 USD 파일이 생성되고, 현재 씬에 로봇 모델이 로드됩니다. 스테이지 패널에서 로봇의 계층 구조와 관절 설정을 확인할 수 있습니다.

## 4. NVIDIA NGC 및 기타 소스에서 에셋 가져오기

*   **NVIDIA NGC (NVIDIA GPU Cloud):**
    *   NGC는 GPU에 최적화된 소프트웨어, 사전 훈련된 AI 모델, 그리고 Isaac Sim에서 사용할 수 있는 3D 에셋(USD 형식) 등을 제공하는 허브입니다.
    *   [NVIDIA NGC 웹사이트](https://ngc.nvidia.com)에서 "Isaac Sim" 또는 "Robotics" 관련 에셋을 검색하여 다운로드할 수 있습니다.
    *   다운로드한 에셋은 보통 USD 형식이므로 콘텐츠 브라우저를 통해 쉽게 가져올 수 있습니다.
*   **Omniverse Launcher의 Exchange:**
    *   Launcher의 "Exchange" 탭에는 NVIDIA 및 서드파티에서 제공하는 다양한 앱, 커넥터, 그리고 에셋 라이브러리가 있습니다.
    *   여기서 필요한 에셋 팩을 설치하면 로컬 Nucleus나 콘텐츠 브라우저를 통해 접근할 수 있게 됩니다.
*   **온라인 3D 모델 저장소:**
    *   Sketchfab, TurboSquid, CGTrader 등 다양한 온라인 마켓플레이스에서 3D 모델을 구매하거나 무료로 다운로드할 수 있습니다.
    *   이러한 모델은 FBX, OBJ 등의 형식일 가능성이 높으므로, Isaac Sim으로 가져오기 전에 USD로 변환하는 과정이 필요할 수 있습니다. Blender와 같은 DCC 툴을 사용하여 USD로 익스포트하거나, Isaac Sim의 내장 임포터 기능을 활용할 수 있습니다.

**에셋 관리 팁:**

*   프로젝트가 커질수록 에셋 관리가 중요해집니다. Nucleus 서버를 활용하여 팀원들과 에셋을 공유하고 버전을 관리하는 것이 좋습니다.
*   자주 사용하는 에셋이나 공용 에셋은 체계적인 폴더 구조로 정리하여 로컬 Nucleus나 파일 시스템에 보관하세요.
*   에셋을 가져올 때는 가능한 한 "참조(Reference)" 방식을 사용하여 원본 파일을 유지하고 씬 파일의 크기를 관리하는 것이 효율적입니다.

이것으로 Isaac Sim 시작하기 튜토리얼 시리즈를 마칩니다. 지금까지 배운 내용을 바탕으로 다양한 에셋을 활용하고 물리 설정을 조절하며 자신만의 로보틱스 시뮬레이션 환경을 구축해 보세요! 다음 단계로는 특정 로봇을 제어하거나, 센서를 추가하고, AI/ML 워크플로우를 통합하는 등의 중급 튜토리얼로 나아갈 수 있습니다.
