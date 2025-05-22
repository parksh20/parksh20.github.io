# Isaac Sim과 ROS 2 Humble 연동

NVIDIA Isaac Sim은 ROS (Robot Operating System) 및 ROS 2와의 강력한 연동 기능을 제공하여, 시뮬레이션 환경과 실제 로봇 애플리케이션 간의 간극을 줄여줍니다. 이 섹션에서는 Isaac Sim을 ROS 2 Humble Hawksbill과 연동하는 방법, 데이터를 주고받는 방법, 그리고 RViz2와 같은 ROS 2 도구를 활용하는 방법을 자세히 다룹니다.

## A. ROS 2 브리지 설정 (ROS 2 Bridge Setup)

Isaac Sim과 ROS 2 간의 통신은 "ROS/ROS2 Bridge" 확장 기능을 통해 이루어집니다. 이 브리지는 Isaac Sim 내부의 데이터와 ROS 2 네트워크 간의 메시지 변환 및 전달을 담당합니다.

### 1. Isaac Sim ROS/ROS2 브리지 확장 기능 설치 및 활성화

기본적으로 Isaac Sim 최신 버전에는 ROS/ROS2 브리지 확장 기능이 포함되어 있거나 쉽게 설치할 수 있도록 제공됩니다.

**활성화 단계:**

1.  **Isaac Sim 실행:** Omniverse Launcher를 통해 Isaac Sim을 실행합니다.
2.  **확장 기능 관리자 열기:** 상단 메뉴 바에서 **Window > Extensions**를 선택하여 확장 기능(Extensions) 관리자 창을 엽니다.
3.  **"ROS" 또는 "ROS2" 검색:** 검색창에 "ROS" 또는 "ROS2"를 입력하여 관련 브리지 확장 기능을 찾습니다. 일반적으로 다음과 같은 이름으로 표시될 수 있습니다 (버전에 따라 다를 수 있음):
    *   `omni.isaac.ros_bridge`
    *   `omni.isaac.ros2_bridge` (최신 버전에서는 이 이름 또는 유사한 이름으로 통합/제공될 가능성이 높습니다)
4.  **확장 기능 활성화:**
    *   찾은 ROS 2 브리지 확장 기능 옆의 토글 스위치를 켜서 활성화(Enable)합니다.
    *   만약 "Install" 버튼이 보인다면, 먼저 설치한 후 활성화합니다.
    *   활성화되면 초록색 점 또는 체크 표시가 나타납니다.
5.  **(선택 사항) 자동 로드 설정:** 확장 기능 이름 옆의 작은 톱니바퀴 아이콘(설정)을 클릭하고 "Autoload"를 체크하면 Isaac Sim 시작 시 자동으로 해당 확장 기능이 로드됩니다.

### 2. 브리지 설정 및 구성 옵션

ROS 2 브리지가 활성화되면, 해당 설정을 통해 통신 방식, 토픽 이름 매핑, 메시지 타입 등을 구성할 수 있습니다.

**설정 접근 방법:**

*   ROS 2 브리지 확장 기능이 활성화된 상태에서, 확장 기능 관리자 창에서 해당 확장 기능을 선택하면 상세 정보 및 설정 옵션이 나타날 수 있습니다.
*   또는, Isaac Sim UI 내 특정 패널(예: Property 패널)이나 메뉴(예: `Isaac Utils > ROS`)를 통해 ROS 브리지 설정을 찾을 수 있습니다. (UI는 버전에 따라 변경될 수 있으므로 공식 문서를 참조하는 것이 좋습니다.)

**주요 구성 옵션:**

*   **ROS Domain ID:**
    *   ROS 2 네트워크에서 사용되는 도메인 ID입니다. 동일한 도메인 ID를 사용하는 ROS 2 노드들만 서로 통신할 수 있습니다.
    *   기본값은 보통 0입니다. Isaac Sim과 통신하려는 외부 ROS 2 시스템의 `ROS_DOMAIN_ID` 환경 변수와 동일하게 설정해야 합니다.
*   **ROS 2 RMW Implementation (미들웨어):**
    *   ROS 2는 다양한 미들웨어(RMW: ROS Middleware) 구현을 지원합니다 (예: Fast DDS, Cyclone DDS).
    *   Isaac Sim 브리지가 사용할 RMW 구현을 선택할 수 있습니다. 외부 ROS 2 시스템과 동일한 RMW를 사용하는 것이 통신 안정성에 도움이 됩니다.
*   **토픽 네임스페이스 (Topic Namespacing):**
    *   Isaac Sim에서 발행하거나 구독하는 ROS 2 토픽에 대한 기본 네임스페이스를 설정할 수 있습니다. 예를 들어, 네임스페이스를 `/isaac_sim`으로 설정하면 모든 토픽이 `/isaac_sim/camera/rgb`와 같이 접두사를 갖게 됩니다.
*   **메시지 타입별 설정 (Per-message type settings):**
    *   카메라, 라이다, 조인트 상태 등 각 데이터 타입별로 ROS 2 브리지 설정을 커스터마이징할 수 있습니다.
    *   **활성화/비활성화 (Enable/Disable):** 특정 데이터 타입의 발행 또는 구독을 전체적으로 켜거나 끌 수 있습니다.
    *   **토픽 이름 (Topic Name):** 각 데이터에 대한 ROS 2 토픽 이름을 지정합니다 (예: `/rgb_image` 대신 `/my_robot/camera/color/image_raw`).
    *   **발행 빈도 (Publish Rate):** 데이터 발행 빈도(Hz)를 설정합니다.
    *   **좌표계 변환 (Frame ID / Coordinate System):** ROS 메시지에 사용될 좌표계(frame ID)를 지정하고, 필요한 경우 Isaac Sim 내부 좌표계와 ROS 표준 좌표계(예: REP-105) 간의 변환을 설정합니다.
    *   **QoS (Quality of Service) 프로파일:** 각 토픽의 통신 품질(신뢰성, 내구성 등)을 설정합니다.

**예시: 카메라 데이터 발행 설정**

만약 로봇에 부착된 "Camera_Left"라는 카메라 프림의 RGB 이미지를 ROS 2로 발행하고 싶다면, 다음과 유사한 설정을 찾거나 구성할 수 있습니다.

1.  Isaac Sim의 `Render Product` (또는 유사한 이름의 카메라 설정)에서 해당 카메라(`Camera_Left`)를 선택합니다.
2.  속성(Property) 패널 또는 ROS 브리지 설정 UI에서 "ROS Publishing" 또는 "ROS2 Publishing" 관련 섹션을 찾습니다.
3.  "RGB Image" 또는 "Color Image" 발행 옵션을 활성화합니다.
4.  **Topic Name:** `/my_robot/camera_left/image_raw` (원하는 토픽 이름으로 변경)
5.  **Frame ID:** `camera_left_optical_frame` (ROS 표준에 맞는 프레임 ID)
6.  **Publish Rate:** `30` (초당 30 프레임)

### 3. 연결 확인

ROS 2 브리지 설정이 완료되면 Isaac Sim과 외부 ROS 2 시스템 간의 연결을 확인할 수 있습니다.

**확인 단계:**

1.  **ROS 2 환경 설정:** Isaac Sim 외부의 터미널에서 ROS 2 Humble 환경을 소싱합니다.
    ```bash
    source /opt/ros/humble/setup.bash
    # 만약 워크스페이스를 사용 중이라면 해당 워크스페이스의 setup.bash도 소싱
    # source ~/my_ros2_ws/install/setup.bash
    ```
    **중요:** Isaac Sim 브리지에서 설정한 `ROS_DOMAIN_ID`와 동일한 `ROS_DOMAIN_ID`를 이 터미널 환경에서도 사용해야 합니다.
    ```bash
    export ROS_DOMAIN_ID=0 # Isaac Sim에서 설정한 ID와 동일하게
    ```
2.  **Isaac Sim에서 시뮬레이션 시작:** Isaac Sim에서 씬을 로드하고 **Play** 버튼을 눌러 시뮬레이션을 시작합니다. 이렇게 해야 ROS 브리지가 활성화되고 데이터 발행/구독이 시작됩니다.
3.  **ROS 2 CLI 도구 사용:**
    *   **토픽 목록 확인:** Isaac Sim이 발행해야 할 토픽이 보이는지 확인합니다.
        ```bash
        ros2 topic list
        ```
        예상되는 토픽(예: `/my_robot/camera_left/image_raw`, `/tf`, `/joint_states`)이 목록에 나타나야 합니다.
    *   **토픽 에코:** 특정 토픽의 메시지가 실제로 발행되는지 확인합니다.
        ```bash
        # 예시: 카메라 이미지 토픽 (메시지가 매우 클 수 있으므로 주의)
        # ros2 topic echo /my_robot/camera_left/image_raw 
        
        # 예시: 조인트 상태 토픽
        ros2 topic echo /joint_states
        ```
        터미널에 해당 토픽의 메시지가 주기적으로 출력되면 성공입니다.
    *   **노드 목록 확인:** Isaac Sim 브리지 관련 노드가 실행 중인지 확인할 수 있습니다. (노드 이름은 버전에 따라 다를 수 있음)
        ```bash
        ros2 node list
        ```

만약 토픽이 보이지 않거나 메시지가 출력되지 않으면 다음 사항을 점검합니다:

*   Isaac Sim ROS 2 브리지 확장 기능이 활성화되어 있는가?
*   Isaac Sim 시뮬레이션이 실행 중(Play 상태)인가?
*   Isaac Sim 브리지 설정에서 해당 데이터 타입의 발행이 활성화되어 있는가?
*   Isaac Sim과 외부 ROS 2 터미널의 `ROS_DOMAIN_ID`가 동일한가?
*   Isaac Sim과 외부 ROS 2 터미널이 동일한 RMW 구현을 사용하고 있는가? (문제가 지속될 경우 시도)
*   방화벽 설정이 ROS 2 통신을 막고 있지는 않은가?

위 단계를 통해 Isaac Sim과 ROS 2 Humble 간의 기본적인 통신 설정을 완료할 수 있습니다. 다음 섹션에서는 구체적인 데이터 타입별 발행 및 구독 방법을 자세히 알아보겠습니다.

---

*(다음 내용: B. Isaac Sim 데이터를 ROS 2 토픽으로 발행)*

## B. Isaac Sim 데이터를 ROS 2 토픽으로 발행 (Publishing Isaac Sim Data to ROS 2 Topics)

ROS 2 브리지가 설정되면 Isaac Sim 내의 다양한 센서 데이터와 로봇 상태 정보를 ROS 2 토픽으로 발행하여 외부 ROS 2 노드들이 활용할 수 있게 됩니다. 각 데이터 타입별 발행 방법과 주요 설정 항목은 다음과 같습니다.

일반적으로 데이터 발행 설정은 다음 두 가지 주요 위치 중 하나 또는 조합으로 이루어집니다:
1.  **프림(Prim)의 속성(Property) 패널:** 데이터를 생성하는 특정 프림(예: 카메라, 라이다 센서, 로봇 관절)을 선택하고, 속성 패널에서 "ROS Publishing" 또는 "ROS2 Publishing" 관련 섹션을 찾아 설정합니다.
2.  **ROS/ROS2 브리지 설정 창:** 확장 기능 관리자에서 ROS 2 브리지를 선택하거나, Isaac Sim 메뉴를 통해 접근 가능한 전역 브리지 설정 창에서 각 데이터 타입별 발행 설정을 관리합니다.

**공통 설정 항목:**
*   **Enable/Disable:** 해당 데이터의 ROS 2 발행 여부를 결정합니다.
*   **Topic Name:** 발행될 ROS 2 토픽의 이름입니다. (예: `/camera/color/image_raw`, `/scan`, `/joint_states`)
*   **Publish Rate (Hz):** 데이터 발행 빈도입니다. 센서의 실제 업데이트 속도와 시뮬레이션 성능을 고려하여 설정합니다.
*   **Frame ID:** 발행되는 메시지의 헤더(header)에 포함될 좌표계(frame) 이름입니다. ROS 표준 좌표계 규칙(REP-105)을 따르는 것이 좋습니다. (예: `camera_link`, `base_link`, `odom`)

### 1. 카메라 이미지 발행 (RGB, Depth)

Isaac Sim의 카메라는 RGB 이미지, 깊이(Depth) 이미지, 포인트 클라우드 등 다양한 데이터를 생성할 수 있으며, 이를 ROS 2 이미지 토픽으로 발행할 수 있습니다.

*   **프림 유형:** Camera 프림 (`Create > Camera`)
*   **ROS 2 메시지 타입:**
    *   RGB 이미지: `sensor_msgs/msg/Image`
    *   깊이 이미지: `sensor_msgs/msg/Image` (일반적으로 16UC1 또는 32FC1 인코딩)
    *   카메라 정보: `sensor_msgs/msg/CameraInfo` (카메라 내부 파라미터, 왜곡 계수 등)
*   **설정 위치:**
    *   해당 카메라 프림을 선택합니다.
    *   속성(Property) 패널에서 "ROS Publishing" 또는 유사한 섹션을 찾습니다.
    *   또는, ROS 2 브리지 설정 창의 "Cameras" 또는 "Image Publishers" 섹션에서 설정합니다.
*   **주요 설정 항목:**
    *   **RGB Image Publisher:**
        *   `Enabled`: 체크하여 RGB 이미지 발행 활성화
        *   `Topic Name`: 예: `/camera/color/image_raw`
        *   `Frame ID`: 예: `camera_color_optical_frame`
    *   **Depth Image Publisher:**
        *   `Enabled`: 체크하여 깊이 이미지 발행 활성화
        *   `Topic Name`: 예: `/camera/depth/image_rect_raw`
        *   `Frame ID`: 예: `camera_depth_optical_frame`
        *   `Depth Encoding`: `16UC1` (밀리미터 단위 정수) 또는 `32FC1` (미터 단위 실수) 선택.
    *   **Camera Info Publisher:**
        *   `Enabled`: 체크하여 카메라 정보 발행 활성화
        *   `Topic Name`: 예: `/camera/color/camera_info` (RGB용), `/camera/depth/camera_info` (깊이용)
        *   `Frame ID`: 해당 이미지 `Frame ID`와 일치시키는 것이 일반적입니다.
*   **사용 예시 (RViz2에서 확인):**
    *   RViz2를 실행하고, "Add" 버튼을 클릭하여 "Image" 디스플레이 타입을 추가합니다.
    *   "Image" 디스플레이의 "Topic" 항목에서 발행 중인 이미지 토픽(예: `/camera/color/image_raw`)을 선택합니다.
    *   "CameraInfo" 토픽도 함께 발행되고 있다면, RViz2의 "Camera" 디스플레이 타입을 사용하여 3D 뷰에 이미지를 투영할 수 있습니다.

### 2. 라이다(Lidar) 스캔 데이터 발행

회전형 또는 고정형 라이다 센서의 스캔 데이터를 ROS 2 `LaserScan` 또는 `PointCloud2` 메시지로 발행할 수 있습니다.

*   **프림 유형:** Lidar Sensor 프림 (`Create > Physics > Lidar Sensor` 또는 유사 경로)
    *   Isaac Sim은 회전형 라이다와 고정형(Solid-state) 라이다를 생성하는 다양한 방법을 제공할 수 있습니다. (예: `RotatingLidarPhysX`, `SolidStateLidarPhysX`)
*   **ROS 2 메시지 타입:**
    *   2D 라이다 스캔: `sensor_msgs/msg/LaserScan`
    *   3D 라이다 포인트 클라우드: `sensor_msgs/msg/PointCloud2`
*   **설정 위치:**
    *   해당 라이다 센서 프림을 선택합니다.
    *   속성 패널 또는 ROS 2 브리지 설정 창의 "Lidar" 또는 "LaserScan Publishers" / "PointCloud Publishers" 섹션에서 설정합니다.
*   **주요 설정 항목 (`LaserScan` 기준):**
    *   `Enabled`: 체크하여 발행 활성화
    *   `Topic Name`: 예: `/scan`
    *   `Frame ID`: 예: `lidar_link`
    *   `Min Range / Max Range`: 라이다 측정 최소/최대 거리
    *   `Horizontal FOV`, `Horizontal Resolution`: 수평 시야각 및 각도 해상도
    *   `Vertical FOV`, `Vertical Resolution` (3D 라이다의 경우)
*   **사용 예시 (RViz2에서 확인):**
    *   RViz2에서 "LaserScan" 또는 "PointCloud2" 디스플레이 타입을 추가합니다.
    *   해당 디스플레이의 "Topic" 항목에서 발행 중인 라이다 토픽(예: `/scan`)을 선택합니다.

### 3. 로봇 조인트 상태 발행 (Publishing Joint States)

로봇의 각 관절(joint)의 현재 각도(position), 속도(velocity), 노력(effort) 등의 상태 정보를 ROS 2 `JointState` 메시지로 발행합니다. 이는 로봇 모델을 RViz2에 시각화하거나 다른 ROS 2 노드에서 로봇의 현재 상태를 파악하는 데 필수적입니다.

*   **프림 유형:** Articulation (관절로 연결된 로봇 모델) 프림. (예: URDF 임포터를 통해 가져온 로봇)
*   **ROS 2 메시지 타입:** `sensor_msgs/msg/JointState`
*   **설정 위치:**
    *   로봇의 루트(root) Articulation 프림을 선택합니다.
    *   속성 패널 또는 ROS 2 브리지 설정 창의 "Articulation" 또는 "Joint State Publishers" 섹션에서 설정합니다.
*   **주요 설정 항목:**
    *   `Enabled`: 체크하여 발행 활성화
    *   `Topic Name`: 예: `/joint_states` (일반적으로 이 이름을 사용)
    *   `Include Joint Names`: 발행되는 `JointState` 메시지에 관절 이름을 포함할지 여부. (일반적으로 포함)
*   **사용 예시:**
    *   RViz2에서 "RobotModel" 디스플레이 타입을 추가하고, `robot_description` 파라미터에 로봇의 URDF 내용을 설정하면 `/joint_states` 토픽을 구독하여 로봇의 자세를 실시간으로 업데이트합니다.

### 4. 오도메트리 정보 발행 (Publishing Odometry)

이동 로봇(Mobile Robot)의 경우, 로봇의 현재 위치(position)와 방향(orientation), 그리고 선속도(linear velocity)와 각속도(angular velocity)를 포함하는 오도메트리 정보를 ROS 2 `Odometry` 메시지로 발행할 수 있습니다.

*   **프림 유형:** 이동 로봇의 베이스(base) 링크 또는 관련 프림. (Differential Drive 컨트롤러 등이 적용된 로봇)
*   **ROS 2 메시지 타입:** `nav_msgs/msg/Odometry`
*   **설정 위치:**
    *   로봇의 오도메트리 정보를 계산하거나 가지고 있는 프림/컨트롤러를 선택합니다.
    *   속성 패널 또는 ROS 2 브리지 설정 창의 "Odometry Publishers" 섹션에서 설정합니다.
*   **주요 설정 항목:**
    *   `Enabled`: 체크하여 발행 활성화
    *   `Topic Name`: 예: `/odom`
    *   `Frame ID`: 오도메트리 데이터의 기준 좌표계 (예: `odom`)
    *   `Child Frame ID`: 로봇의 베이스 좌표계 (예: `base_link`)
*   **사용 예시:**
    *   ROS 2 내비게이션 스택(Nav2)은 `/odom` 토픽을 입력으로 받아 로봇의 위치를 추정합니다.
    *   RViz2에서 "Odometry" 디스플레이 타입을 추가하여 로봇의 이동 경로와 자세를 시각화할 수 있습니다.

### 5. TF (Transform) 데이터 발행

로봇의 각 링크(link)들 간의 상대적인 자세 변환 정보, 그리고 로봇과 월드(world) 또는 맵(map) 좌표계 간의 변환 정보를 ROS 2 TF(Transform) 시스템을 통해 발행합니다. TF는 ROS 2에서 다양한 좌표계 간의 데이터 변환에 핵심적인 역할을 합니다.

*   **프림 유형:** Articulation (로봇 모델), 또는 TF 정보를 발행하도록 설정된 특정 프림.
*   **ROS 2 메시지 타입:** `tf2_msgs/msg/TFMessage` (일반적으로 `/tf` 및 `/tf_static` 토픽으로 발행됨)
*   **설정 위치:**
    *   로봇의 루트 Articulation 프림 또는 전역 ROS 2 브리지 설정에서 TF 발행 관련 옵션을 찾습니다.
*   **주요 설정 항목:**
    *   `Enabled`: 체크하여 TF 발행 활성화
    *   **Static TF Publisher (`/tf_static`):** 로봇 모델 내에서 변경되지 않는 정적인 링크 간의 변환 정보를 발행합니다. (예: `base_link` -> `lidar_link`의 고정된 변환)
    *   **Dynamic TF Publisher (`/tf`):** 시뮬레이션 중에 변경될 수 있는 동적인 변환 정보를 발행합니다. (예: `odom` -> `base_link` 변환, 로봇 팔의 각 링크 간 변환)
    *   `Publish Rate`: TF 발행 빈도.
*   **사용 예시:**
    *   RViz2는 TF 데이터를 사용하여 로봇 모델, 센서 데이터 등을 올바른 위치와 방향으로 시각화합니다.
    *   대부분의 ROS 2 패키지는 TF를 통해 특정 좌표계의 데이터를 다른 좌표계로 변환하여 사용합니다.

**중요 고려 사항:**

*   **좌표계 일치 (Frame ID Consistency):** 발행하는 모든 데이터의 `frame_id`는 일관성 있게 사용되어야 하며, 로봇의 URDF에 정의된 링크 이름과 일치시키는 것이 일반적입니다. ROS REP-103 (Standard Units of Measure and Coordinate Conventions) 및 REP-105 (Coordinate Frames for Mobile Platforms) 문서를 참조하여 표준 좌표계 규칙을 따르는 것이 좋습니다.
*   **성능:** 너무 많은 데이터를 매우 높은 빈도로 발행하면 Isaac Sim 및 ROS 2 시스템 모두에 부하를 줄 수 있습니다. 필요한 데이터만 적절한 빈도로 발행하도록 설정하는 것이 중요합니다.
*   **Isaac Sim 내부 vs. ROS 시간 동기화:** 시뮬레이션 시간과 실제 ROS 시간을 동기화하는 옵션(`use_sim_time` 파라미터)을 고려해야 할 수 있습니다. Isaac Sim에서 시뮬레이션 시간을 기준으로 데이터를 발행하고, ROS 2 측에서도 시뮬레이션 시간을 사용하도록 설정하면(예: `ros2 param set /<node_name> use_sim_time true`), 보다 일관된 데이터 처리가 가능합니다.

이러한 설정을 통해 Isaac Sim의 풍부한 시뮬레이션 데이터를 ROS 2 생태계로 가져와 다양한 로봇 애플리케이션 개발 및 테스트에 활용할 수 있습니다.

---

*(다음 내용: C. ROS 2 토픽을 구독하여 Isaac Sim 제어)*

## C. ROS 2 토픽을 구독하여 Isaac Sim 제어 (Subscribing to ROS 2 Topics to Control Isaac Sim)

Isaac Sim은 외부 ROS 2 노드로부터 제어 명령을 받아 시뮬레이션 내의 로봇이나 다른 객체를 움직이게 할 수 있습니다. 이는 실제 로봇에서 사용하는 것과 동일한 ROS 2 제어 노드를 시뮬레이션 환경에 적용하여 테스트하고 개발하는 데 매우 유용합니다.

일반적으로 제어 명령 구독 설정은 다음 위치에서 이루어집니다:
1.  **제어 대상 프림(Prim)의 속성(Property) 패널:** 제어하려는 로봇의 관절(Articulation) 프림이나 특정 액추에이터 프림을 선택하고, 속성 패널에서 "ROS Subscriptions" 또는 "ROS2 Subscriptions" 관련 섹션을 찾아 설정합니다.
2.  **ROS/ROS2 브리지 설정 창:** 전역 브리지 설정에서 제어 관련 구독 설정을 관리할 수도 있습니다.

**공통 설정 항목:**
*   **Enable/Disable:** 해당 제어 명령 구독 여부를 결정합니다.
*   **Topic Name:** 구독할 ROS 2 토픽의 이름입니다. (예: `/cmd_vel`, `/joint_trajectory_controller/joint_trajectory`)
*   **ROS 2 Message Type:** 구독할 메시지의 타입입니다. (예: `geometry_msgs/msg/Twist`, `trajectory_msgs/msg/JointTrajectory`)

### 1. 로봇 제어 명령 구독

#### a. 차동 구동 로봇 (Differential Drive Robots)

바퀴 달린 이동 로봇(예: TurtleBot, Jackal)을 제어하는 가장 일반적인 방법은 선속도(linear velocity)와 각속도(angular velocity)를 포함하는 `geometry_msgs/msg/Twist` 메시지를 사용하는 것입니다.

*   **구독 대상 프림:** 로봇의 베이스(base)에 적용된 차동 구동 컨트롤러 관련 프림 또는 로봇의 Articulation 루트 프림.
    *   Isaac Sim에서는 로봇 프림에 `Differential Drive` 컨트롤러를 추가하여 이를 쉽게 설정할 수 있습니다 (`Create > Physics > Controllers > Differential Drive`).
*   **ROS 2 메시지 타입:** `geometry_msgs/msg/Twist`
*   **설정 위치:**
    *   차동 구동 컨트롤러 프림 또는 로봇 프림을 선택합니다.
    *   속성 패널에서 "ROS Subscriptions" 또는 "ROS2 Twist Subscriber" (또는 유사한 이름) 섹션을 찾습니다.
*   **주요 설정 항목:**
    *   `Enabled`: 체크하여 `/cmd_vel` (또는 지정한 토픽) 구독 활성화.
    *   `Topic Name`: 기본값은 보통 `/cmd_vel`이지만, 필요에 따라 변경 가능합니다. (예: `/my_robot/cmd_vel`)
    *   `Max Linear Speed / Max Angular Speed`: 로봇의 최대 선속도 및 각속도를 설정하여 Twist 메시지의 값을 스케일링할 수 있습니다.
*   **사용 예시 (외부 ROS 2 노드에서 제어):**
    *   **`teleop_twist_keyboard` 사용:**
        ```bash
        # 새 터미널에서 ROS 2 환경 소싱 및 teleop_twist_keyboard 실행
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=0 # Isaac Sim과 동일한 ID
        ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/my_robot # 네임스페이스 사용 시
        # 또는 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/my_robot/cmd_vel # 토픽 리매핑
        ```
        위 명령 실행 후 키보드(예: `i` 키: 전진, `,` 키: 후진, `j`/`l` 키: 회전)를 사용하여 Isaac Sim 내의 로봇을 움직일 수 있습니다.
    *   **직접 토픽 발행:**
        ```bash
        # 선속도 0.5 m/s로 전진
        ros2 topic pub --rate 1 /my_robot/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ```

#### b. 매니퓰레이터 (Manipulators / Robot Arms)

로봇 팔과 같은 매니퓰레이터는 각 관절의 목표 각도(position), 속도(velocity), 또는 노력(effort)을 지정하여 제어합니다. `trajectory_msgs/msg/JointTrajectory` 메시지는 시간에 따른 관절 경로를 정의하는 데 사용됩니다.

*   **구독 대상 프림:** 로봇의 Articulation 루트 프림.
*   **ROS 2 메시지 타입:**
    *   `trajectory_msgs/msg/JointTrajectory` (경로 기반 제어)
    *   `control_msgs/msg/FollowJointTrajectory` (액션 기반 제어 - `ros2_control`과 함께 사용)
    *   `std_msgs/msg/Float64MultiArray` (개별 관절 직접 제어 - 간단한 테스트용)
*   **설정 위치:**
    *   로봇의 Articulation 루트 프림을 선택합니다.
    *   속성 패널에서 "ROS Subscriptions" 또는 "ROS2 Joint Command Subscriber" (또는 유사한 이름) 섹션을 찾습니다.
*   **주요 설정 항목 (`JointTrajectory` 기준):**
    *   `Enabled`: 체크하여 구독 활성화.
    *   `Topic Name`: 예: `/joint_trajectory_controller/joint_trajectory` (MoveIt2와 같은 플래너가 발행하는 기본 토픽) 또는 `/my_robot_arm_controller/joint_trajectory`.
    *   `Joint Names`: 제어할 관절들의 이름을 URDF에 정의된 순서대로 정확히 입력해야 합니다. (예: `['joint1', 'joint2', 'shoulder_pan_joint', 'elbow_lift_joint', ...]`)
*   **사용 예시 (MoveIt2 또는 커스텀 노드에서 제어):**
    *   **MoveIt2 연동:** Isaac Sim에서 로봇 모델과 `ros2_control` 설정을 올바르게 구성하면, MoveIt2를 사용하여 모션 플래닝을 수행하고 그 결과를 `/joint_trajectory_controller/joint_trajectory` 토픽으로 발행하여 Isaac Sim 내의 로봇 팔을 움직일 수 있습니다. (이는 고급 설정에 해당)
    *   **간단한 `JointTrajectory` 메시지 발행 (Python 스크립트 예시):**
        ```python
        # (ROS 2 Python 클라이언트 라이브러리 rclpy 사용)
        # import rclpy
        # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        # ... (노드 초기화 및 퍼블리셔 생성) ...
        # trajectory_msg = JointTrajectory()
        # trajectory_msg.joint_names = ['joint1', 'joint2', ...]
        # point = JointTrajectoryPoint()
        # point.positions = [1.0, -0.5, ...] # 목표 관절 각도 (라디안)
        # point.time_from_start.sec = 2 # 2초 안에 도달
        # trajectory_msg.points.append(point)
        # publisher.publish(trajectory_msg)
        ```

### 2. 기타 액추에이터 제어 (Controlling Other Actuators)

로봇의 그리퍼(gripper), 선형 액추에이터 등 특정 목적을 가진 액추에이터도 ROS 2 토픽을 통해 제어할 수 있습니다.

*   **구독 대상 프림:** 해당 액추에이터를 나타내는 관절(Joint) 프림 또는 Articulation 프림.
*   **ROS 2 메시지 타입:**
    *   그리퍼: `control_msgs/action/GripperCommand` (액션 기반), 또는 간단한 경우 `std_msgs/msg/Float64` (관절 목표 위치).
    *   선형 액추에이터: `std_msgs/msg/Float64` (목표 위치/개방 정도).
*   **설정 위치:**
    *   해당 액추에이터 프림을 선택하고 속성 패널에서 ROS 구독 설정을 찾습니다.
*   **주요 설정 항목:**
    *   `Enabled`, `Topic Name`, `Message Type` 등을 액추에이터의 특성과 제어 방식에 맞게 설정합니다.
    *   예를 들어, 그리퍼의 특정 관절을 직접 제어하려면 해당 관절에 "Joint Target Follower" (또는 유사한 이름의 컴포넌트)를 추가하고, 이 컴포넌트가 `std_msgs/msg/Float64` 타입의 토픽을 구독하도록 설정할 수 있습니다.

**제어 명령 구독 시 고려 사항:**

*   **관절 이름 및 순서:** 매니퓰레이터 제어 시 `JointTrajectory` 메시지의 `joint_names` 필드는 Isaac Sim 로봇 모델의 관절 이름 및 순서와 정확히 일치해야 합니다. URDF 파일과 Isaac Sim 내부의 관절 순서를 확인하세요.
*   **제어 모드 (Control Mode):** Isaac Sim에서 로봇 관절은 위치(position), 속도(velocity), 또는 노력(effort/torque) 제어 모드로 설정될 수 있습니다. ROS 2 토픽으로 전달되는 명령은 해당 관절의 제어 모드와 호환되어야 합니다. 예를 들어, 위치 제어 모드인 관절에 토크 명령을 보내면 제대로 작동하지 않을 수 있습니다.
    *   Articulation 프림의 속성에서 각 관절의 `Drive Type` (예: `Position`, `Velocity`)을 설정할 수 있습니다.
*   **단위 (Units):** ROS 2 메시지의 단위(예: 라디안, 미터)와 Isaac Sim 내부에서 사용하는 단위가 일치하는지 확인해야 합니다. (예: 각도는 라디안, 거리는 미터).
*   **충돌 방지:** 외부에서 제어 명령을 보낼 때, Isaac Sim 내부의 물리 엔진은 충돌을 감지하고 반응합니다. 하지만 제어 명령 자체가 충돌을 유발할 수 있으므로, 외부 제어 노드(예: MoveIt2)에서 충돌 없는 경로를 계획하는 것이 중요합니다.

이러한 방식으로 외부 ROS 2 노드에서 발행하는 다양한 제어 명령을 구독하여 Isaac Sim 내의 로봇과 환경을 실시간으로 상호작용시킬 수 있습니다.

---

*(다음 내용: D. RViz2 활용)*

## D. RViz2 활용 (Using RViz2)

RViz2는 ROS 2 데이터를 3D로 시각화하는 강력한 도구입니다. Isaac Sim에서 발행하는 다양한 센서 데이터, 로봇 모델, TF 변환 등을 RViz2에서 확인하여 시뮬레이션 환경과 ROS 2 시스템 간의 연동 상태를 효과적으로 디버깅하고 모니터링할 수 있습니다.

### 1. Isaac Sim의 센서 데이터를 RViz2에서 시각화

Isaac Sim에서 발행하는 카메라 이미지, 라이다 스캔, 포인트 클라우드 등의 센서 데이터를 RViz2에서 시각화하는 방법입니다.

**사전 준비:**

1.  **Isaac Sim 설정:**
    *   Isaac Sim에서 ROS 2 브리지가 활성화되어 있고, 원하는 센서 데이터(카메라, 라이다 등)가 올바른 토픽 이름과 `frame_id`로 발행되도록 설정되어 있는지 확인합니다. (이전 "B. Isaac Sim 데이터를 ROS 2 토픽으로 발행" 섹션 참조)
    *   TF 데이터도 함께 발행되도록 설정합니다. (`/tf` 및 `/tf_static`)
    *   Isaac Sim 시뮬레이션을 **Play** 상태로 만듭니다.
2.  **ROS 2 환경 설정:**
    *   새 터미널을 열고 ROS 2 Humble 환경을 소싱합니다.
        ```bash
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=0 # Isaac Sim과 동일한 ID
        ```
3.  **RViz2 실행:**
    ```bash
    rviz2
    ```

**RViz2에서 센서 데이터 추가 및 설정:**

*   **글로벌 옵션 (Global Options):**
    *   RViz2 왼쪽 패널의 "Displays" 창 상단에 있는 "Global Options"를 확장합니다.
    *   **Fixed Frame:** 시각화의 기준이 될 좌표계를 선택합니다. 일반적으로 로봇의 `odom` 프레임, `map` 프레임, 또는 로봇의 `base_link` 프레임을 선택합니다. Isaac Sim에서 발행하는 TF를 기준으로 존재하는 프레임을 선택해야 합니다.
*   **카메라 이미지 시각화:**
    1.  Displays 패널 하단의 "Add" 버튼 클릭.
    2.  "By display type" 탭에서 "Image"를 선택하고 "OK" 클릭.
    3.  새로 추가된 "Image" 디스플레이를 확장합니다.
    4.  **Topic:** Isaac Sim에서 발행하는 카메라 이미지 토픽(예: `/camera/color/image_raw`)을 선택합니다.
    5.  **Transport Hint:** `raw` (일반적) 또는 `compressed` (압축 이미지 사용 시) 등을 선택합니다.
    6.  화면에 이미지가 나타나면 성공입니다.
    *   **CameraInfo 토픽 활용:** 만약 `sensor_msgs/msg/CameraInfo` 토픽도 함께 발행된다면, "Camera" 디스플레이 타입을 사용하여 3D 뷰에 이미지를 올바르게 투영할 수 있습니다.
        1.  "Add" -> "Camera" 선택.
        2.  **Image Topic:** 이미지 토픽 선택.
        3.  **Camera Info Topic:** 해당 이미지의 `CameraInfo` 토픽(예: `/camera/color/camera_info`) 선택.
*   **라이다 스캔 (LaserScan) 시각화:**
    1.  "Add" -> "LaserScan" 선택.
    2.  새로 추가된 "LaserScan" 디스플레이를 확장합니다.
    3.  **Topic:** Isaac Sim에서 발행하는 `sensor_msgs/msg/LaserScan` 토픽(예: `/scan`)을 선택합니다.
    4.  **Size (m):** 각 스캔 포인트의 크기를 조절합니다.
    5.  **Style:** `Points`, `Lines` 등 표시 스타일을 선택합니다.
    6.  3D 뷰에 라이다 스캔 데이터가 나타나야 합니다.
*   **포인트 클라우드 (PointCloud2) 시각화:**
    1.  "Add" -> "PointCloud2" 선택.
    2.  새로 추가된 "PointCloud2" 디스플레이를 확장합니다.
    3.  **Topic:** Isaac Sim에서 발행하는 `sensor_msgs/msg/PointCloud2` 토픽(예: `/lidar/pointcloud`)을 선택합니다.
    4.  **Style:** `Points`가 일반적입니다.
    5.  **Size (Pixels / Meters):** 포인트 크기를 조절합니다.
    6.  **Color Transformer:** 포인트 클라우드의 색상 표현 방식을 선택합니다 (예: `Intensity`, `AxisColor`).
*   **TF (좌표계 변환) 시각화:**
    1.  "Add" -> "TF" 선택.
    2.  새로 추가된 "TF" 디스플레이를 확장합니다.
    3.  **Show Names:** 각 프레임의 이름을 표시할지 여부.
    4.  **Show Axes:** 각 프레임의 축을 표시할지 여부.
    5.  **Show Arrows:** 프레임 간의 관계를 화살표로 표시할지 여부.
    6.  **Marker Scale:** 표시되는 축이나 이름의 크기를 조절합니다.
    7.  이를 통해 Isaac Sim에서 정의되고 발행된 모든 좌표계(로봇 링크, 센서 프레임, odom 등)와 그 관계를 시각적으로 확인할 수 있습니다.

### 2. 로봇 모델을 RViz2에 표시

Isaac Sim에서 시뮬레이션 중인 로봇의 모델을 RViz2에 표시하려면, 로봇의 URDF(Unified Robot Description Format) 정보와 현재 조인트 상태가 필요합니다.

**사전 준비:**

1.  **로봇 URDF 파일 준비:**
    *   RViz2에 표시할 로봇의 URDF 파일이 필요합니다. Isaac Sim으로 로봇을 가져올 때 사용한 URDF 파일 또는 Isaac Sim에서 생성/변환된 USD에서 다시 추출한 URDF일 수 있습니다.
2.  **`robot_state_publisher` 실행:**
    *   `robot_state_publisher` 노드는 URDF 파일과 `sensor_msgs/msg/JointState` 토픽을 입력으로 받아 로봇의 각 링크에 대한 TF 변환 정보를 계산하여 `/tf` 및 `/tf_static` 토픽으로 발행합니다.
    *   일반적으로 로봇별 ROS 2 실행 파일(launch file) 내에 포함되어 실행됩니다.
    *   수동으로 실행하는 예시:
        ```bash
        # 로봇 URDF 파일 경로를 파라미터로 전달
        ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /path/to/your/robot.urdf)"
        # 또는 launch 파일 사용
        # ros2 launch your_robot_description_pkg view_robot.launch.py
        ```
        위 명령어에서 `/path/to/your/robot.urdf`는 실제 URDF 파일 경로로 대체해야 합니다.
3.  **Isaac Sim 설정:**
    *   Isaac Sim에서 해당 로봇의 `JointState` 데이터가 `/joint_states` (또는 `robot_state_publisher`가 구독하도록 설정된 다른 토픽 이름) 토픽으로 발행되도록 설정합니다.
    *   TF 데이터도 Isaac Sim에서 발행하도록 설정되어 있다면, `robot_state_publisher`와 충돌하지 않도록 주의해야 합니다. 일반적으로 `robot_state_publisher`가 로봇 내부 링크 간의 TF를 담당하고, Isaac Sim은 로봇의 월드 좌표계(예: `odom` -> `base_link`) TF를 발행하도록 역할을 분담할 수 있습니다. 또는 Isaac Sim의 TF 발행을 최소화하고 `robot_state_publisher`에 의존할 수도 있습니다.
    *   Isaac Sim 시뮬레이션을 **Play** 상태로 만듭니다.

**RViz2에서 로봇 모델 추가 및 설정:**

1.  **RViz2 실행:** (이미 실행 중이라면 생략)
    ```bash
    rviz2
    ```
2.  **글로벌 옵션 설정:**
    *   **Fixed Frame:** `odom` 또는 로봇의 루트 링크(예: `base_link`)를 선택합니다.
3.  **RobotModel 디스플레이 추가:**
    1.  Displays 패널 하단의 "Add" 버튼 클릭.
    2.  "By display type" 탭에서 "RobotModel"을 선택하고 "OK" 클릭.
    3.  새로 추가된 "RobotModel" 디스플레이를 확장합니다.
    4.  **Description Source:** `Topic`을 선택합니다. (또는 `File`을 선택하고 URDF 파일 경로를 직접 지정할 수도 있지만, `robot_state_publisher`를 사용할 경우 `Topic`이 일반적입니다.)
    5.  **Description Topic:** `robot_description`을 선택합니다. (`robot_state_publisher`가 이 토픽 이름으로 URDF 내용을 파라미터 서버에 올립니다.)
    6.  **TF Prefix:** 필요한 경우 TF 접두사를 설정합니다 (일반적으로 비워둠).
4.  **결과 확인:**
    *   RViz2의 3D 뷰에 로봇 모델이 나타나야 합니다.
    *   Isaac Sim에서 로봇을 움직이면, 발행되는 `/joint_states`와 TF 데이터를 통해 RViz2의 로봇 모델도 실시간으로 따라서 움직여야 합니다.

**문제 해결 팁:**

*   **로봇 모델이 나타나지 않거나 이상하게 보이는 경우:**
    *   `Fixed Frame`이 올바르게 설정되었는지 확인합니다.
    *   `robot_state_publisher`가 정상적으로 실행 중이고, `robot_description` 파라미터가 올바르게 로드되었는지 확인합니다. (`ros2 param get /robot_state_publisher robot_description`)
    *   Isaac Sim에서 `/joint_states` 토픽이 올바른 관절 이름과 데이터로 발행되는지 확인합니다. (`ros2 topic echo /joint_states`)
    *   TF 데이터가 올바르게 발행되고 있는지 확인합니다. (RViz2의 TF 디스플레이 또는 `ros2 run tf2_tools tf_echo <source_frame> <target_frame>` 사용)
    *   URDF 파일 자체에 오류가 없는지 확인합니다 (`check_urdf <urdf_file>`).
*   **센서 데이터가 로봇 모델과 정렬되지 않는 경우:**
    *   센서 데이터의 `frame_id`와 로봇 모델의 링크 이름, 그리고 TF 데이터가 모두 일관되게 설정되었는지 확인합니다. 모든 데이터는 동일한 TF 트리를 공유해야 합니다.

RViz2는 Isaac Sim과 ROS 2 연동 개발 과정에서 매우 유용한 시각화 및 디버깅 도구입니다. 다양한 디스플레이 타입을 활용하여 원하는 정보를 효과적으로 모니터링하고 문제를 해결할 수 있습니다.

---

*(다음 내용: E. 예제 프로젝트)*

## E. 예제 프로젝트: ROS 2를 통해 제어되는 간단한 로봇 내비게이션

이 예제 프로젝트에서는 Isaac Sim에서 차동 구동 로봇을 로드하고, ROS 2를 통해 키보드로 로봇을 제어하며, RViz2에서 로봇의 움직임과 센서 데이터를 시각화하는 과정을 단계별로 안내합니다.

**목표:**

*   Isaac Sim에 TurtleBot3 로봇을 로드합니다.
*   ROS 2 브리지를 설정하여 로봇의 라이다 스캔, 오도메트리, TF 데이터를 발행하고, `/cmd_vel` 토픽을 구독하여 로봇을 제어합니다.
*   `teleop_twist_keyboard` 노드를 사용하여 키보드로 로봇을 움직입니다.
*   RViz2에서 로봇 모델, 라이다 스캔, TF, 오도메트리 정보를 시각화합니다.

**사전 준비:**

1.  **Isaac Sim 설치 및 ROS 2 Humble 설치 완료:** (이전 섹션들 참조)
2.  **TurtleBot3 ROS 2 패키지 설치 (선택 사항이지만 권장):**
    RViz2에서 TurtleBot3 모델을 정확히 보려면 관련 패키지가 있는 것이 좋습니다.
    ```bash
    sudo apt update
    sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
    ```
    이 패키지에는 TurtleBot3의 URDF 파일과 `robot_state_publisher` 설정이 포함된 launch 파일 등이 들어있습니다.
3.  **ROS 2 환경 소싱:** 모든 터미널에서 ROS 2 환경을 소싱해야 합니다.
    ```bash
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0 # 또는 Isaac Sim에서 설정한 ID
    ```

---

**단계 1: Isaac Sim 환경 설정**

1.  **Isaac Sim 실행 및 새 스테이지 생성:**
    *   Omniverse Launcher를 통해 Isaac Sim을 실행합니다.
    *   **File > New**를 선택하여 빈 스테이지를 만듭니다.
2.  **지면 추가:**
    *   **Create > Physics > Ground Plane**을 선택하여 지면을 추가합니다.
3.  **조명 추가:**
    *   **Create > Light > Distant Light**를 선택하여 조명을 추가합니다.
4.  **TurtleBot3 로봇 로드:**
    *   Isaac Sim은 다양한 방법으로 로봇을 로드할 수 있습니다. 여기서는 NVIDIA에서 제공하는 기본 에셋을 사용하거나, URDF 임포터를 사용하는 방법을 가정합니다.
    *   **방법 A: NVIDIA 제공 에셋 사용 (권장)**
        1.  **Content Browser** 패널을 엽니다.
        2.  `Isaac/Robots/TurtleBot3/` 경로로 이동합니다. (경로는 버전에 따라 다를 수 있습니다. `turtlebot3_burger.usd` 또는 유사한 파일을 찾으세요.)
        3.  `turtlebot3_burger.usd` 파일을 뷰포트로 드래그 앤 드롭합니다.
    *   **방법 B: URDF 임포터 사용** (만약 TurtleBot3 URDF 파일이 있다면)
        1.  **Isaac Utils > Importer > URDF Importer**를 선택합니다.
        2.  **Input File:** TurtleBot3 URDF 파일 (`turtlebot3_burger.urdf`) 경로를 지정합니다.
        3.  **Import Physics, Create Articulation** 옵션을 체크합니다.
        4.  "IMPORT"를 클릭합니다.
    *   로봇이 지면 위에 올바르게 배치되었는지 확인하고, 필요하면 이동 도구(W)를 사용하여 위치를 조정합니다.
5.  **ROS 2 브리지 확장 기능 활성화:**
    *   **Window > Extensions**에서 `omni.isaac.ros2_bridge` (또는 유사 이름) 확장 기능을 찾아 활성화합니다.
6.  **로봇에 대한 ROS 2 브리지 설정:**
    *   스테이지(Stage) 패널에서 로드된 TurtleBot3 로봇의 루트 프림(예: `turtlebot3_burger`)을 선택합니다.
    *   속성(Property) 패널을 아래로 스크롤하여 "ROS Publishing" 또는 "ROS2" 관련 섹션을 찾습니다. 다음과 같이 설정하거나 확인합니다. (UI 및 옵션 이름은 버전에 따라 약간 다를 수 있습니다.)

    *   **TF (Transform Tree):**
        *   `Enabled`: 체크
        *   `Publish World to Base TF`: 체크 (또는 `odom` to `base_footprint` 발행 설정)
        *   `Base Frame ID`: `base_footprint` (TurtleBot3의 기본 베이스 프레임)
        *   `Odom Frame ID`: `odom`
    *   **JointState:**
        *   `Enabled`: 체크
        *   `Topic Name`: `/joint_states`
    *   **Lidar (LaserScan):** (TurtleBot3 Burger에는 LDS-01 라이다가 부착되어 있음)
        *   로봇 모델 내의 라이다 프림(예: `LDS_01_Lidar`)을 찾아 선택합니다.
        *   속성 패널에서 "ROS Publishing" 또는 "ROS2 Lidar Publisher" 섹션을 찾습니다.
        *   `Enabled`: 체크
        *   `Topic Name`: `/scan`
        *   `Frame ID`: `base_scan` (TurtleBot3 라이다의 기본 프레임)
    *   **Odometry:**
        *   TurtleBot3 로봇의 루트 프림을 다시 선택합니다.
        *   "ROS Publishing" 섹션 또는 별도의 "Odometry" 섹션에서 다음을 설정합니다.
        *   `Enabled`: 체크
        *   `Topic Name`: `/odom`
        *   `Odom Frame ID`: `odom`
        *   `Base Frame ID`: `base_footprint`
    *   **Twist (로봇 제어 구독):**
        *   TurtleBot3 로봇 루트 프림에 `Differential Drive` 컨트롤러가 이미 적용되어 있을 수 있습니다. 없다면 추가합니다:
            1.  로봇 루트 프림 선택.
            2.  **Add > Physics > Controllers > Differential Drive**.
        *   생성된 `DifferentialDrive` (또는 유사 이름) 프림을 선택합니다.
        *   속성 패널에서 "ROS Subscriptions" 또는 "ROS2 Twist Subscriber" 섹션을 찾습니다.
        *   `Enabled`: 체크
        *   `Topic Name`: `/cmd_vel`
        *   `Max Linear Speed / Max Angular Speed`: 적절한 값 설정 (예: 선속도 1.0 m/s, 각속도 2.0 rad/s).

7.  **시뮬레이션 시작:**
    *   Isaac Sim 인터페이스 하단의 **Play** 버튼을 클릭하여 시뮬레이션을 시작합니다.

---

**단계 2: ROS 2 환경에서 로봇 제어 및 시각화**

이제 별도의 터미널에서 ROS 2 명령을 사용하여 로봇을 제어하고 RViz2로 시각화합니다. 각 터미널에서 ROS 2 환경을 소싱하는 것을 잊지 마세요.

1.  **터미널 1: `robot_state_publisher` 실행 (RViz2 로봇 모델 표시용)**
    ```bash
    source /opt/ros/humble/setup.bash
    # TurtleBot3 패키지가 설치되어 있다면 launch 파일 사용 가능
    ros2 launch turtlebot3_bringup robot.launch.py
    # 또는 수동으로 URDF 경로 지정 (URDF 파일이 있다면)
    # ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /path/to/your/turtlebot3_burger.urdf)"
    ```
    만약 `turtlebot3_bringup robot.launch.py` 실행 시 에러가 발생하면, TurtleBot3 패키지가 제대로 설치되지 않았거나 환경 설정에 문제가 있을 수 있습니다. 이 경우, Isaac Sim에서 발행하는 TF만으로도 RViz2에서 센서 데이터를 보는 데는 충분할 수 있지만, `RobotModel` 표시는 어려울 수 있습니다. `robot_state_publisher`는 로봇의 각 링크 간의 TF를 발행하는 데 주로 사용됩니다. Isaac Sim이 이미 `/tf`를 통해 유사한 정보를 발행하고 있다면, 역할이 중복될 수 있으니 주의합니다. 여기서는 RViz2에서 `RobotModel`을 정확히 보기 위해 실행한다고 가정합니다.

2.  **터미널 2: `teleop_twist_keyboard` 실행 (로봇 제어용)**
    ```bash
    source /opt/ros/humble/setup.bash
    # 기본적으로 teleop_twist_keyboard는 /cmd_vel 토픽으로 발행합니다.
    # Isaac Sim의 TurtleBot3도 /cmd_vel을 구독하도록 설정했으므로 아래 명령으로 실행합니다.
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # 만약 Isaac Sim의 Twist 구독 토픽을 변경했다면 (예: /my_robot/cmd_vel), 
    # 아래와 같이 리매핑하여 실행해야 합니다:
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/my_robot/cmd_vel
    ```
    이 터미널에 포커스를 두고 키보드(i, j, l, , 등)를 누르면 지정된 토픽으로 `geometry_msgs/msg/Twist` 메시지가 발행됩니다. Isaac Sim의 TurtleBot3가 해당 토픽을 구독하고 있으므로 움직여야 합니다.

3.  **터미널 3: RViz2 실행 (시각화용)**
    ```bash
    source /opt/ros/humble/setup.bash
    rviz2
    ```
    *   **RViz2 설정:**
        1.  **Global Options > Fixed Frame:** `odom` 또는 `base_footprint`를 선택합니다.
        2.  **Add > By display type > RobotModel:** 추가하고 "Description Topic"을 `robot_description`으로 설정합니다. (만약 `robot_state_publisher`가 정상 실행 중이라면)
        3.  **Add > By display type > TF:** 추가하여 모든 좌표계 변환을 시각화합니다.
        4.  **Add > By display type > LaserScan:** 추가하고 "Topic"을 `/scan`으로 설정합니다. 라이다 데이터가 보여야 합니다.
        5.  **Add > By display type > Odometry:** 추가하고 "Topic"을 `/odom`으로 설정합니다. 로봇의 이동 경로가 표시될 수 있습니다.
        6.  필요에 따라 각 디스플레이의 색상, 크기 등을 조절합니다.

**결과 확인:**

*   Isaac Sim에서 TurtleBot3가 Play 상태로 실행 중이어야 합니다.
*   터미널 2에서 키보드를 입력하면 Isaac Sim 내의 TurtleBot3가 해당 방향으로 움직여야 합니다.
*   RViz2 화면에 TurtleBot3 모델이 보이고, 키보드 입력에 따라 실시간으로 움직여야 합니다.
*   RViz2에 라이다 스캔 데이터가 로봇 주변에 표시되어야 합니다.
*   TF 디스플레이를 통해 `odom`, `base_footprint`, `base_scan` 등의 좌표계가 올바르게 연결되어 있는지 확인할 수 있습니다.

---

이 예제 프로젝트는 Isaac Sim과 ROS 2 간의 기본적인 연동 방식을 보여줍니다. 이를 바탕으로 더 복잡한 센서(예: 카메라)를 추가하거나, ROS 2 내비게이션 스택(Nav2)과 연동하여 자율 주행을 시뮬레이션하는 등 다양한 확장이 가능합니다. 중요한 것은 각 컴포넌트(Isaac Sim 브리지, ROS 2 노드, RViz2)의 설정과 토픽 이름, `frame_id` 등을 일관성 있게 맞추는 것입니다.

---

*(다음 내용: III. Isaac Lab 와 ROS 2 Humble 연동)*
