# Isaac Sim 고급 토픽: OmniGraph 심층 분석

Isaac Sim의 기본 사용법에 익숙해졌다면, 이제 OmniGraph를 통해 더욱 복잡하고 정교한 시뮬레이션 로직과 로봇 행동을 구현하는 방법을 알아볼 차례입니다. OmniGraph는 NVIDIA Omniverse의 핵심 기술 중 하나로, 노드 기반의 비주얼 프로그래밍 환경을 제공하여 사용자가 코드를 직접 작성하지 않고도 복잡한 시스템을 설계하고 제어할 수 있도록 지원합니다.

이 섹션에서는 OmniGraph의 내부 작동 원리, 커스텀 노드 제작 방법, 그리고 Python 스크립트와의 연동을 통해 OmniGraph의 활용도를 극대화하는 방법을 심층적으로 다룹니다.

## 1. OmniGraph의 작동 원리 및 장점

**OmniGraph란?**

OmniGraph (또는 Action Graph)는 Isaac Sim 내에서 다양한 작업을 자동화하고, 로봇의 행동을 정의하며, 시뮬레이션 이벤트를 처리하기 위한 강력한 비주얼 스크립팅 도구입니다. 사용자는 그래프 편집기 내에서 다양한 기능을 수행하는 "노드(Node)"들을 연결하여 데이터 흐름과 실행 순서를 시각적으로 구성할 수 있습니다.

**작동 원리:**

1.  **노드 (Nodes):** 각 노드는 특정 작업(예: 수학 연산, 프림 속성 읽기/쓰기, 이벤트 감지, Python 스크립트 실행)을 수행하는 기본 단위입니다. 노드는 입력(Input) 및 출력(Output) 어트리뷰트(Attribute)를 가집니다.
2.  **어트리뷰트 (Attributes):** 노드의 입력 또는 출력 포트로, 데이터를 받거나 전달하는 통로 역할을 합니다. 어트리뷰트는 특정 데이터 타입(예: 정수, 실수, 불리언, 벡터, 프림 경로)을 가집니다.
3.  **연결 (Connections):** 한 노드의 출력 어트리뷰트를 다른 노드의 입력 어트리뷰트로 연결하여 데이터 흐름을 정의합니다. 이를 통해 노드 간에 정보가 전달되고, 특정 순서대로 작업이 실행됩니다.
4.  **실행 흐름 (Execution Flow):**
    *   **Push 방식:** 데이터가 입력되면 해당 노드가 실행되고, 그 결과가 연결된 다음 노드로 전달되어 연쇄적으로 실행됩니다. (데이터 기반 실행)
    *   **Tick 방식:** 특정 이벤트(예: 시뮬레이션 스텝 업데이트, 사용자 입력)가 발생할 때마다 그래프의 특정 부분(또는 전체)이 실행됩니다. `OnTick` 노드와 같은 이벤트 노드가 실행의 시작점이 됩니다.
5.  **그래프 (Graphs):** 여러 노드와 그 연결로 구성된 전체 로직의 집합입니다. 하나의 OmniGraph는 특정 목적(예: 로봇 센서 데이터 처리, 특정 행동 제어)을 수행합니다.

**OmniGraph의 장점:**

*   **시각적 프로그래밍:** 코드 작성 없이 로직을 시각적으로 구성하므로, 프로그래밍에 익숙하지 않은 사용자도 비교적 쉽게 접근하고 이해할 수 있습니다.
*   **모듈성 및 재사용성:** 잘 정의된 기능을 수행하는 노드들을 조합하여 복잡한 시스템을 구축할 수 있습니다. 커스텀 노드(아래에서 설명)나 서브그래프(Subgraph)를 만들어 로직을 모듈화하고 다른 프로젝트에서 재사용하기 용이합니다.
*   **실시간 인터랙션 및 디버깅:** 그래프의 실행 상태와 각 노드의 데이터 값을 실시간으로 모니터링할 수 있어 디버깅이 용이합니다.
*   **Python과의 강력한 통합:** Python 스크립트를 실행하는 노드를 통해 OmniGraph의 기능을 무한히 확장할 수 있으며, 기존 Python 코드와 쉽게 연동할 수 있습니다.
*   **이벤트 기반 아키텍처:** 시뮬레이션 내의 다양한 이벤트(예: 물리 업데이트, 키보드 입력, 프림 충돌)에 반응하여 특정 로직을 실행할 수 있어 동적인 시뮬레이션 환경 구축에 유리합니다.
*   **USD와의 통합:** OmniGraph는 USD(Universal Scene Description)와 깊이 통합되어 있어, 씬 내의 프림 속성을 직접 읽고 수정하며 씬의 상태에 따라 동적으로 반응하는 로직을 쉽게 구현할 수 있습니다.

## 2. 커스텀 노드 생성 및 사용

Isaac Sim은 다양한 기본 노드를 제공하지만, 특정 프로젝트에 필요한 특수한 기능을 수행하는 노드가 필요할 때가 있습니다. 이 경우, 사용자는 Python을 사용하여 직접 커스텀 OmniGraph 노드를 만들 수 있습니다.

**커스텀 노드 생성 단계 (Python 기반):**

1.  **노드 정의 파일 생성:**
    *   Python 파일(`.py`)을 생성하여 커스텀 노드의 로직을 정의합니다.
    *   이 파일은 일반적으로 Omniverse 확장(Extension)의 일부로 관리되거나, 프로젝트의 특정 스크립트 폴더에 위치할 수 있습니다.
2.  **필수 라이브러리 임포트:**
    ```python
    import omni.graph.core as og
    import omni.usd
    # 필요한 경우 다른 Isaac Sim 또는 Python 라이브러리 임포트
    # 예: from pxr import Gf
    ```
3.  **노드 클래스 정의:**
    *   `og.Node` (또는 특정 기본 노드 타입)를 상속받는 Python 클래스를 정의합니다.
    *   클래스 내에 노드의 입력/출력 어트리뷰트, 내부 상태, 그리고 핵심 로직을 포함하는 `compute` 메서드 등을 정의합니다.

    ```python
    class MyCustomNode:
        # 노드 메타데이터 (OmniGraph 편집기에서 표시될 정보)
        @staticmethod
        def node_meta_info():
            # 노드 타입 이름 (고유해야 함)
            return {"node_type_name": "omni.isaac.examples.MyCustomNode", 
                    # UI에 표시될 이름
                    "display_name": "My Custom Logic Node",
                    # 노드 설명
                    "description": "A custom node that performs a specific operation."}

        # 입력 어트리뷰트 정의
        @staticmethod
        def inputs_meta_info():
            return {
                "execIn": {"type": "execution", "description": "Execution input."},
                "inputValueA": {"type": "float", "default_value": 0.0, "description": "Input value A."},
                "inputValueB": {"type": "float", "default_value": 0.0, "description": "Input value B."}
            }

        # 출력 어트리뷰트 정의
        @staticmethod
        def outputs_meta_info():
            return {
                "execOut": {"type": "execution", "description": "Execution output."},
                "outputValue": {"type": "float", "description": "Result of the operation."}
            }
        
        # 노드의 핵심 로직 (데이터베이스에서 실제 값을 계산)
        def compute(self, db: og.Database) -> bool:
            # 입력 값 가져오기
            val_a = db.inputs.inputValueA
            val_b = db.inputs.inputValueB
            
            # 계산 수행
            result = val_a + val_b  # 예시: 간단한 덧셈
            
            # 출력 값 설정
            db.outputs.outputValue = result
            
            # execOut 실행 신호 전달 (Push 방식 노드의 경우)
            db.outputs.execOut = og.ExecutionAttributeState.PUSH
            
            return True # 성공적으로 계산 완료

        # (선택 사항) 노드 초기화 시 호출되는 함수
        def initialize(self, graph_context: og.GraphContext, node_path: str):
            pass

        # (선택 사항) 노드 제거 시 호출되는 함수
        def release(self, graph_context: og.GraphContext, node_path: str):
            pass
    ```

4.  **노드 등록:**
    *   작성한 노드 클래스를 OmniGraph 시스템에 등록해야 편집기에서 사용할 수 있습니다. 이는 보통 확장 기능의 `on_startup` 메서드나 별도의 등록 스크립트를 통해 이루어집니다.
    ```python
    # 확장 기능의 on_startup 등에서 호출
    def register_custom_nodes():
        try:
            og.register_node_type(MyCustomNode.node_meta_info()["node_type_name"], 
                                  MyCustomNode.inputs_meta_info(), 
                                  MyCustomNode.outputs_meta_info(), 
                                  MyCustomNode)
            print("MyCustomNode registered successfully.")
        except Exception as e:
            print(f"Failed to register MyCustomNode: {e}")
    
    # 확장 기능의 on_shutdown 등에서 호출
    def unregister_custom_nodes():
        try:
            og.unregister_node_type(MyCustomNode.node_meta_info()["node_type_name"])
            print("MyCustomNode unregistered successfully.")
        except Exception as e:
            print(f"Failed to unregister MyCustomNode: {e}")

    ```
    *   **참고:** `og.register_node_type`의 인자는 노드 클래스의 구조(예: `compute` 메서드의 유무, 상태 저장 방식)에 따라 달라질 수 있습니다. 위 예시는 가장 일반적인 "Push" 방식 노드의 등록 방법입니다. 상태를 가지는 노드(`StatefulNode`)나 비동기 노드 등은 다른 등록 방식이나 기본 클래스를 사용할 수 있습니다.

**커스텀 노드 사용:**

1.  커스텀 노드가 포함된 확장 기능을 활성화하거나 등록 스크립트를 실행합니다.
2.  OmniGraph 편집기(Action Graph 또는 Flow)를 엽니다.
3.  편집기 내 노드 생성 메뉴(보통 Tab 키 또는 우클릭)에서 정의한 `display_name` (예: "My Custom Logic Node")을 검색하여 그래프에 추가합니다.
4.  다른 노드와 마찬가지로 입력/출력 어트리뷰트를 연결하여 사용합니다.

## 3. 복잡한 로봇 행동 및 시뮬레이션 로직 구현

OmniGraph를 사용하면 여러 센서 데이터를 조합하고, 조건에 따라 다른 행동을 하며, 시간에 따라 변하는 복잡한 로봇 행동 및 시뮬레이션 로직을 구현할 수 있습니다.

**구현 전략:**

*   **계층적 그래프 설계 (Hierarchical Graphs):**
    *   하나의 거대한 그래프 대신, 특정 기능을 수행하는 여러 개의 서브그래프(Subgraph) 또는 연결된 그래프로 로직을 분리합니다.
    *   예를 들어, "센서 데이터 처리 그래프", "경로 계획 그래프", "관절 제어 그래프" 등으로 나누어 관리하면 가독성과 유지보수성이 향상됩니다.
*   **상태 머신 (State Machines):**
    *   로봇의 행동 상태(예: "대기", "이동 중", "작업 중", "오류")를 정의하고, 각 상태에서 수행할 로직과 상태 간 전환 조건을 OmniGraph로 구현합니다.
    *   `If` 노드, `Switch` 노드, 불리언 로직 노드 등을 사용하여 상태 전환 로직을 만듭니다.
*   **이벤트 기반 로직:**
    *   `OnTick` (매 시뮬레이션 스텝), `OnKeyboardInput` (키보드 입력 감지), `OnPrimColliderHit` (프림 충돌 감지), `OnTimer` (주기적 실행) 등 다양한 이벤트 노드를 활용하여 특정 상황에 반응하는 로직을 만듭니다.
*   **데이터 변환 및 처리:**
    *   센서 데이터(예: 카메라 이미지, 라이다 포인트 클라우드)를 직접 처리하거나, Python 노드를 통해 NumPy, OpenCV 등의 라이브러리를 활용하여 데이터를 분석하고 변환한 후 로봇 제어에 사용합니다.
    *   벡터 연산, 행렬 변환, 삼각 함수 등 수학 노드를 활용하여 좌표 변환이나 운동학 계산 등을 수행할 수 있습니다.
*   **로봇 관절 제어:**
    *   `Articulation Controller` 노드, `Drive Articulation` 노드, `Set Articulation Target` 노드 등을 사용하여 로봇의 개별 관절 또는 전체 관절 그룹을 제어합니다.
    *   PID 제어기 로직을 OmniGraph로 직접 구현하거나, Python 노드를 통해 커스텀 제어 알고리즘을 적용할 수 있습니다.
*   **USD 속성 활용:**
    *   `Read Prim Attribute` 노드로 씬 내 프림의 속성(예: 위치, 방향, 커스텀 속성)을 읽고, `Write Prim Attribute` 노드로 속성 값을 변경하여 시뮬레이션 환경과 동적으로 상호작용합니다.

**예시: 간단한 장애물 회피 로직 (개념)**

1.  **`OnTick` 노드:** 매 시뮬레이션 스텝마다 실행을 시작합니다.
2.  **라이다 센서 데이터 읽기 노드:** 로봇 전방의 라이다 데이터를 읽습니다. (Python 노드 또는 전용 센서 노드)
3.  **데이터 처리 노드:** 라이다 데이터에서 가장 가까운 장애물까지의 거리를 계산합니다.
4.  **`If` 노드 (조건 분기):**
    *   만약 (장애물 거리 < 안전 거리) 이면:
        *   **회전 명령 생성 노드:** 로봇을 특정 방향으로 회전시키는 Twist 명령(선속도 0, 각속도 설정)을 생성합니다.
    *   아니면 (Else):
        *   **직진 명령 생성 노드:** 로봇을 앞으로 직진시키는 Twist 명령(선속도 설정, 각속도 0)을 생성합니다.
5.  **`Drive Robot` (또는 `Publish Twist Command`) 노드:** 생성된 Twist 명령을 로봇 컨트롤러에 전달합니다.

## 4. Python과의 연동

OmniGraph는 Python 스크립트와 매우 긴밀하게 연동될 수 있어, OmniGraph의 시각적인 장점과 Python의 강력한 프로그래밍 능력을 함께 활용할 수 있습니다.

**연동 방법:**

*   **`Python Script` 노드:**
    *   OmniGraph 내에서 가장 직접적으로 Python 코드를 실행하는 방법입니다.
    *   노드의 입력 어트리뷰트를 통해 Python 스크립트로 데이터를 전달하고, 스크립트 내에서 계산이나 로직 처리를 수행한 후, 그 결과를 노드의 출력 어트리뷰트를 통해 다른 노드로 전달할 수 있습니다.
    *   **사용 예시:**
        1.  `Python Script` 노드를 그래프에 추가합니다.
        2.  노드의 속성 창에서 `Script Path`에 실행할 Python 파일 경로를 지정하거나, `Script String`에 직접 Python 코드를 작성합니다.
        3.  노드의 입력/출력 어트리뷰트를 정의하고, Python 스크립트 내에서 `db.inputs.<input_name>`으로 입력 값을 읽고 `db.outputs.<output_name> = result` 형태로 출력 값을 설정합니다. (여기서 `db`는 `compute` 함수의 인자로 전달되는 `og.Database` 객체입니다.)

*   **커스텀 Python 노드:** (위에서 설명한 방법)
    *   복잡하거나 재사용성이 높은 Python 로직은 별도의 커스텀 노드로 만들어 관리하는 것이 더 효율적입니다.
*   **OmniGraph API를 사용한 Python 스크립트에서의 그래프 제어:**
    *   Python 스크립트에서 OmniGraph API (`omni.graph.core`, `omni.graph.scriptnode`)를 사용하여 OmniGraph를 생성, 수정, 실행 제어할 수 있습니다.
    *   예를 들어, 특정 조건에서 Python 스크립트가 특정 OmniGraph를 로드하여 실행시키거나, 그래프 내 노드의 속성 값을 동적으로 변경할 수 있습니다.
    ```python
    import omni.graph.core as og
    
    # 그래프 경로 가져오기 (예: 현재 스테이지에서 특정 경로의 그래프)
    # graph_path = "/World/MyRobot/ActionGraph" 
    # graph = og.get_graph_by_path(graph_path)
    
    # 만약 그래프가 있다면, 특정 노드의 속성 값 변경
    # if graph:
    #     node_path = graph_path + "/MyNode" # 그래프 내 노드 경로
    #     og.set_attribute(node_path + ".inputs:myInputValue", 123.45)
    
    #     # 그래프 실행 (Push 방식 노드의 경우, 시작 노드의 execIn을 PUSH로 설정)
    #     # start_node_path = graph_path + "/MyStartNode"
    #     # og.set_execution_attribute_state(start_node_path + ".inputs:execIn", og.ExecutionAttributeState.PUSH)
    ```
*   **Omniverse Commands를 통한 연동:**
    *   Python 스크립트에서 Omniverse Command를 사용하여 OmniGraph 노드를 생성하거나 연결하는 등의 작업을 수행할 수 있습니다. 이는 UI에서 수동으로 하는 작업을 스크립트로 자동화하는 데 유용합니다.

**Python 연동의 장점:**

*   **NumPy, SciPy, OpenCV 등 외부 라이브러리 활용:** 복잡한 수학 계산, 이미지 처리, 데이터 분석 등을 Python 라이브러리를 통해 수행하고 그 결과를 OmniGraph에서 활용할 수 있습니다.
*   **기존 Python 코드 재사용:** 이미 작성된 로봇 제어 로직, AI 모델 추론 코드 등을 `Python Script` 노드나 커스텀 노드를 통해 OmniGraph와 쉽게 통합할 수 있습니다.
*   **파일 입출력, 네트워크 통신 등 고급 기능 구현:** OmniGraph만으로는 구현하기 어려운 파일 시스템 접근, 네트워크 통신 등의 기능을 Python으로 구현하여 연동합니다.

OmniGraph는 Isaac Sim의 강력한 기능들을 조합하고 확장하여 사용자가 원하는 고유한 시뮬레이션 환경과 로봇 애플리케이션을 구축할 수 있게 해주는 핵심 도구입니다. 시각적인 인터페이스와 Python의 유연성을 결합하여 그 가능성을 탐색해 보시기 바랍니다.
