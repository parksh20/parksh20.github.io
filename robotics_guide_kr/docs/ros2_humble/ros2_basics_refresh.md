# ROS 2 Humble 기본 (ROS 2 Humble Basics - 간략한 복습)

이 섹션에서는 Isaac Sim 및 Isaac Lab과의 연동을 시작하기 전에 ROS 2 Humble의 핵심 개념을 간략하게 복습하고, 설치 방법 및 기본 CLI 도구 사용법을 안내합니다. 이미 ROS 2에 익숙하신 분들은 이 섹션을 건너뛰셔도 좋습니다.

## 1. ROS 2 핵심 개념 (Core Concepts)

ROS 2 (Robot Operating System 2)는 로봇 애플리케이션 개발을 위한 오픈 소스 프레임워크입니다. ROS 2는 분산 시스템 아키텍처를 가지며, 여러 "노드(Node)"들이 서로 통신하며 작업을 수행합니다.

*   **노드 (Nodes):**
    *   ROS 2 시스템에서 실행되는 최소 단위의 프로세스입니다. 각 노드는 특정 목적을 가진 작은 프로그램을 실행합니다 (예: 센서 데이터 처리, 모터 제어, 경로 계획 등).
    *   하나의 로봇 시스템은 여러 개의 노드로 구성될 수 있으며, 각 노드는 독립적으로 실행되고 서로 통신합니다.
    *   예: `/camera_driver` 노드, `/lidar_processor` 노드, `/motor_controller` 노드.
*   **토픽 (Topics):**
    *   노드 간의 비동기식 단방향 메시지 통신 채널입니다.
    *   **발행자(Publisher)** 노드는 특정 토픽으로 메시지를 지속적으로 "발행(publish)"하고, 해당 토픽을 **구독자(Subscriber)** 노드는 메시지를 "구독(subscribe)"하여 수신합니다.
    *   일대다(one-to-many), 다대일(many-to-one), 다대다(many-to-many) 통신이 가능합니다.
    *   메시지는 미리 정의된 데이터 구조(예: `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`)를 가집니다.
    *   예: `/image_raw` 토픽 (카메라 이미지 데이터), `/cmd_vel` 토픽 (로봇 이동 속도 명령).
*   **서비스 (Services):**
    *   노드 간의 동기식 양방향 요청/응답(request/reply) 통신 방식입니다.
    *   **서비스 클라이언트(Service Client)** 노드가 특정 서비스에 요청을 보내면, **서비스 서버(Service Server)** 노드가 요청을 처리하고 응답을 반환합니다.
    *   요청이 처리될 때까지 클라이언트는 대기(blocking)하는 것이 일반적입니다.
    *   예: `/set_robot_pose` 서비스 (로봇의 특정 자세로 이동 요청), `/compute_inverse_kinematics` 서비스 (역기구학 계산 요청).
*   **액션 (Actions):**
    *   장시간 실행되는 작업을 위한 비동기식 양방향 통신 방식입니다. 서비스와 유사하지만, 중간 피드백(feedback)을 제공하고 작업 취소(cancellation) 기능을 지원합니다.
    *   **액션 클라이언트(Action Client)**가 목표(goal)를 액션 서버에 전송하면, **액션 서버(Action Server)**는 목표를 수행하면서 주기적으로 피드백을 보내고, 작업이 완료되면 결과(result)를 반환합니다.
    *   예: `/navigate_to_pose` 액션 (특정 지점까지 로봇 자율 주행), `/follow_joint_trajectory` 액션 (로봇 팔 관절 경로 추종).
*   **파라미터 (Parameters):**
    *   노드의 설정을 외부에서 변경하거나 조회할 수 있도록 하는 값들입니다.
    *   각 노드는 자신만의 파라미터를 가질 수 있으며, 실행 중에 동적으로 파라미터 값을 변경할 수 있습니다.
    *   예: 카메라 노드의 `/camera_info_url` 파라미터 (카메라 보정 파일 경로), 모터 제어 노드의 `/max_velocity` 파라미터 (최대 속도).
*   **메시지 (Messages - `.msg` 파일):**
    *   토픽, 서비스, 액션 통신에 사용되는 데이터의 구조를 정의합니다.
    *   기본 데이터 타입(정수, 실수, 문자열, 배열 등)과 다른 메시지 타입을 조합하여 복잡한 데이터 구조를 만들 수 있습니다.
    *   패키지 내 `msg` 디렉토리에 `.msg` 파일로 정의됩니다. (예: `std_msgs/msg/String.msg`, `geometry_msgs/msg/Point.msg`)
*   **서비스 정의 (Service Definitions - `.srv` 파일):**
    *   서비스 통신에서 요청(request)과 응답(reply) 부분의 데이터 구조를 정의합니다.
    *   하나의 `.srv` 파일은 `---` 구분자를 기준으로 요청 부분과 응답 부분으로 나뉩니다.
    *   패키지 내 `srv` 디렉토리에 `.srv` 파일로 정의됩니다.
*   **액션 정의 (Action Definitions - `.action` 파일):**
    *   액션 통신에서 목표(goal), 결과(result), 피드백(feedback) 부분의 데이터 구조를 정의합니다.
    *   하나의 `.action` 파일은 `---` 구분자를 사용하여 세 부분으로 나뉩니다.
    *   패키지 내 `action` 디렉토리에 `.action` 파일로 정의됩니다.

## 2. Ubuntu 22.04에 ROS 2 Humble Hawksbill 설치

ROS 2 Humble Hawksbill은 Ubuntu 22.04 (Jammy Jellyfish)를 공식 지원합니다. 설치는 주로 Debian 패키지를 통해 이루어집니다.

**공식 설치 문서:**

가장 정확하고 최신 정보는 항상 ROS 2 공식 문서를 참조하는 것이 좋습니다.
*   **ROS 2 Humble 설치 가이드 (영문):** [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

**요약된 설치 단계 (데스크탑 풀 버전 기준):**

아래는 공식 문서의 주요 단계를 요약한 것입니다. 설치 전에 반드시 위 링크의 공식 문서를 확인하세요.

1.  **UTF-8 로케일 설정 확인 및 설정:**
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    ```
2.  **ROS 2 APT 저장소 추가:**
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **ROS 2 패키지 설치:**
    ```bash
    sudo apt update
    sudo apt upgrade # (선택 사항이지만 권장)
    # 데스크탑 풀 버전 (RViz, 데모, 튜토리얼 포함):
    sudo apt install ros-humble-desktop 
    # 또는 ROS 기본 버전 (GUI 도구 제외):
    # sudo apt install ros-humble-ros-base
    ```
4.  **환경 설정 스크립트 소싱:**
    ROS 2 명령어를 사용하려면 터미널 세션마다 환경 설정 스크립트를 소싱(source)해야 합니다.
    ```bash
    # 현재 터미널에만 적용
    source /opt/ros/humble/setup.bash
    ```
    새 터미널을 열 때마다 자동으로 적용되도록 `.bashrc` 파일에 추가하는 것이 편리합니다.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
5.  **(선택 사항) 개발 도구 설치:**
    ROS 2 패키지를 직접 빌드하거나 개발하려면 추가 도구가 필요합니다.
    ```bash
    sudo apt install ros-dev-tools
    ```
6.  **(선택 사항) Colcon 자동완성 설치:**
    Colcon은 ROS 2의 빌드 시스템입니다. 자동완성 기능을 설치하면 편리합니다.
    ```bash
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```

**설치 확인:**
간단한 예제(예: `talker`와 `listener`)를 실행하여 설치가 올바르게 되었는지 확인할 수 있습니다.

*   터미널 1:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 run demo_nodes_cpp talker
    ```
*   터미널 2:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 run demo_nodes_py listener
    ```
    `listener` 터미널에 `talker`가 발행하는 "Hello World" 메시지가 나타나면 성공입니다.

## 3. 기본 CLI 도구 사용법

ROS 2는 명령줄 인터페이스(Command Line Interface, CLI) 도구를 통해 시스템을 검사하고 상호작용할 수 있는 다양한 기능을 제공합니다. 모든 명령어는 `ros2`로 시작합니다.

*   **`ros2 run <package_name> <executable_name>`:**
    *   패키지 내의 실행 파일(노드)을 실행합니다.
    *   예: `ros2 run turtlesim turtlesim_node` (turtlesim 시뮬레이터 실행)
*   **`ros2 node list`:**
    *   현재 실행 중인 모든 노드의 목록을 보여줍니다.
*   **`ros2 node info <node_name>`:**
    *   특정 노드의 상세 정보(구독 중인 토픽, 발행 중인 토픽, 서비스, 액션 등)를 보여줍니다.
    *   예: `ros2 node info /turtlesim`
*   **`ros2 topic list`:**
    *   현재 활성화된 모든 토픽의 목록을 보여줍니다.
    *   `-t` 옵션을 추가하면 토픽의 메시지 타입도 함께 표시됩니다. (예: `ros2 topic list -t`)
*   **`ros2 topic info <topic_name>`:**
    *   특정 토픽의 상세 정보(메시지 타입, 발행자 수, 구독자 수)를 보여줍니다.
    *   예: `ros2 topic info /turtle1/cmd_vel`
*   **`ros2 topic echo <topic_name>`:**
    *   특정 토픽으로 발행되는 메시지의 내용을 실시간으로 화면에 출력합니다.
    *   예: `ros2 topic echo /turtle1/pose`
*   **`ros2 topic pub <topic_name> <message_type> '<args>'`:**
    *   특정 토픽으로 메시지를 한 번 또는 주기적으로 발행합니다.
    *   `--once` 옵션: 메시지를 한 번만 발행.
    *   `--rate 1` 옵션: 1Hz (1초에 한 번) 주기로 메시지 발행.
    *   `<args>`는 YAML 형식의 메시지 내용입니다.
    *   예 (turtlesim 거북이 전진):
        ```bash
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ```
*   **`ros2 service list`:**
    *   현재 사용 가능한 모든 서비스의 목록을 보여줍니다.
    *   `-t` 옵션을 추가하면 서비스 타입도 함께 표시됩니다.
*   **`ros2 service type <service_name>`:**
    *   특정 서비스의 타입을 보여줍니다.
*   **`ros2 service call <service_name> <service_type> '<args>'`:**
    *   특정 서비스에 요청을 보내고 응답을 받습니다.
    *   예 (turtlesim 거북이 생성):
        ```bash
        ros2 service call /spawn turtlesim/srv/Spawn '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'
        ```
*   **`ros2 action list`:**
    *   현재 사용 가능한 모든 액션의 목록을 보여줍니다.
    *   `-t` 옵션을 추가하면 액션 타입도 함께 표시됩니다.
*   **`ros2 action info <action_name>`:**
    *   특정 액션의 상세 정보를 보여줍니다.
*   **`ros2 action send_goal <action_name> <action_type> '<goal_args>'`:**
    *   특정 액션 서버에 목표를 전송합니다.
    *   `--feedback` 옵션을 추가하면 중간 피드백도 함께 출력됩니다.
    *   예 (turtlesim 거북이 회전 액션):
        ```bash
        # (turtlesim_node가 실행 중이고, 해당 액션 서버가 있다면)
        ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute '{theta: 1.57}' --feedback
        ```
*   **`ros2 param list <node_name>`:**
    *   특정 노드의 파라미터 목록을 보여줍니다.
*   **`ros2 param get <node_name> <parameter_name>`:**
    *   특정 노드의 파라미터 값을 가져옵니다.
*   **`ros2 param set <node_name> <parameter_name> <value>`:**
    *   특정 노드의 파라미터 값을 설정합니다.
*   **`ros2 bag record <topic1> <topic2> ...`:**
    *   지정된 토픽의 메시지를 `rosbag` 파일로 기록합니다.
    *   `-a` 옵션: 모든 활성 토픽을 기록.
*   **`ros2 bag play <bag_file_name>`:**
    *   기록된 `rosbag` 파일을 재생하여 메시지를 다시 발행합니다.
*   **`ros2 pkg list`:**
    *   설치된 모든 ROS 2 패키지 목록을 보여줍니다.
*   **`ros2 pkg prefix <package_name>`:**
    *   특정 패키지가 설치된 경로를 보여줍니다.

이러한 기본 개념과 CLI 도구에 익숙해지면 Isaac Sim 및 Isaac Lab과의 ROS 2 연동을 이해하고 활용하는 데 큰 도움이 됩니다. 다음 섹션에서는 Isaac Sim과 ROS 2 Humble을 연동하는 구체적인 방법을 살펴보겠습니다.
