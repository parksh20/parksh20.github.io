# Isaac Sim 고급 토픽: 외부 애플리케이션 연동

Isaac Sim은 강력한 단독 시뮬레이션 도구이지만, 그 잠재력은 외부 애플리케이션 및 라이브러리와 연동될 때 더욱 확장됩니다. Python 스크립팅 환경과 Omniverse 플랫폼의 개방성을 활용하여 Isaac Sim을 다양한 외부 시스템과 통합하고 데이터를 주고받을 수 있습니다.

이 섹션에서는 Python 스크립트를 사용하여 외부 라이브러리(예: 데이터 처리, 커스텀 시각화)와 연동하는 방법, 그리고 TCP/IP와 같은 표준 통신 프로토콜을 사용하여 Isaac Sim과 다른 애플리케이션 간의 실시간 데이터 교환을 구현하는 방법에 대해 설명합니다.

## 1. Python 스크립트를 통한 외부 라이브러리/애플리케이션 연동

Isaac Sim의 Python 환경은 표준 Python 인터프리터와 거의 동일하게 작동하므로, pip를 통해 설치할 수 있는 대부분의 Python 라이브러리를 Isaac Sim 내 스크립트에서 직접 임포트하여 사용할 수 있습니다. 이를 통해 데이터 분석, 고급 수학 계산, 커스텀 UI 개발, AI 모델 추론 등 다양한 외부 기능을 Isaac Sim 시뮬레이션과 통합할 수 있습니다.

**연동 가능한 라이브러리 예시:**

*   **데이터 처리 및 분석:**
    *   **NumPy:** 고성능 다차원 배열 객체와 이를 다루는 도구를 제공합니다. 센서 데이터, 물리 상태 정보 등을 NumPy 배열로 변환하여 복잡한 수학적 계산 및 통계 분석을 수행할 수 있습니다.
    *   **Pandas:** 데이터 조작 및 분석을 위한 강력한 자료구조(DataFrame 등)를 제공합니다. 시뮬레이션 결과를 CSV, Excel 등으로 저장하거나 외부 데이터를 불러와 시뮬레이션 파라미터로 사용하는 데 유용합니다.
    *   **SciPy:** 과학 및 공학 분야의 여러 수치 계산 루틴(최적화, 보간법, 신호 처리, 통계 등)을 제공합니다.
*   **커스텀 시각화:**
    *   **Matplotlib:** 정적, 애니메이션, 인터랙티브 시각화를 위한 포괄적인 라이브러리입니다. 시뮬레이션 중 특정 데이터(예: 로봇 관절 토크, 엔드 이펙터 경로)를 실시간으로 그래프로 그리거나, 시뮬레이션 종료 후 결과 데이터를 다양한 형태로 시각화할 수 있습니다.
    *   **Plotly/Dash:** 인터랙티브 웹 기반 대시보드 및 시각화를 만드는 데 사용됩니다. 시뮬레이션 파라미터를 웹 UI로 제어하거나 결과를 실시간으로 웹에 표시하는 복잡한 애플리케이션 구축에 유용합니다.
    *   **OpenCV (cv2):** 실시간 컴퓨터 비전 라이브러리로, Isaac Sim에서 생성된 카메라 이미지를 OpenCV로 가져와 객체 감지, 특징 추출, 이미지 필터링 등의 처리를 수행하고 그 결과를 다시 시뮬레이션에 반영하거나 외부로 전송할 수 있습니다.
*   **AI/머신러닝 프레임워크:**
    *   **PyTorch/TensorFlow:** 학습된 AI 모델(예: 비전 모델, 강화학습 에이전트)을 Isaac Sim 환경 내에서 직접 실행(추론)하고, 시뮬레이션 데이터를 입력으로 사용하거나 모델의 출력을 로봇 제어에 활용할 수 있습니다.
*   **GUI 라이브러리:**
    *   **Tkinter, PyQt, Kivy:** Isaac Sim의 기본 UI 시스템(omni.ui) 외에, Python에서 널리 사용되는 GUI 라이브러리를 사용하여 독립적인 제어판이나 데이터 모니터링 창을 만들 수 있습니다. 다만, omni.ui와 직접 통합되지 않을 수 있어 별도의 프로세스로 실행해야 할 수도 있습니다.

**연동 방법 (일반적인 절차):**

1.  **라이브러리 설치:**
    *   Isaac Sim에 포함된 Python 환경 또는 사용자가 설정한 가상 환경에 `pip`를 사용하여 필요한 라이브러리를 설치합니다.
    *   Isaac Sim 실행 환경(예: Kit 터미널 또는 Isaac Sim과 함께 제공되는 Python 실행 파일)에서 `pip install <library_name>` 명령을 실행합니다.
    *   예: `pip install numpy matplotlib opencv-python`
2.  **Python 스크립트에서 임포트 및 사용:**
    *   Isaac Sim의 Python Script 노드, 커스텀 확장 기능, 또는 독립 실행형 Python 스크립트(Kit 앱으로 실행) 내에서 일반 Python 프로그램처럼 해당 라이브러리를 임포트하여 사용합니다.

    ```python
    # 예시: NumPy와 Matplotlib를 사용한 간단한 데이터 분석 및 시각화
    import omni.usd
    import numpy as np
    import matplotlib.pyplot as plt
    
    # Isaac Sim의 현재 스테이지에서 로봇의 위치 데이터를 가져온다고 가정
    # (실제 데이터 수집 로직은 더 복잡할 수 있음)
    def get_robot_positions_over_time():
        # 이 함수는 시뮬레이션 루프에서 주기적으로 호출되어 
        # 로봇의 X, Y 위치를 리스트로 반환한다고 가정
        # 예: return [(time1, x1, y1), (time2, x2, y2), ...]
        # 여기서는 임의의 데이터 생성
        times = np.linspace(0, 10, 100)
        x_positions = np.sin(times)
        y_positions = np.cos(times)
        return times, x_positions, y_positions

    # 데이터 수집 (실제로는 시뮬레이션 루프 내에서 수행)
    # collected_data_times, collected_data_x, collected_data_y = [], [], []
    # for _ in range(100): # 100 스텝 동안 데이터 수집 가정
    #     # new_time, new_x, new_y = get_current_robot_pose() 
    #     # collected_data_times.append(new_time)
    #     # collected_data_x.append(new_x)
    #     # collected_data_y.append(new_y)
    #     pass # 실제 데이터 수집 로직 필요
        
    # 임시 데이터 사용
    times, x_positions, y_positions = get_robot_positions_over_time()
    
    # NumPy를 사용한 간단한 분석
    mean_x = np.mean(x_positions)
    max_y = np.max(y_positions)
    print(f"Mean X position: {mean_x}, Max Y position: {max_y}")
    
    # Matplotlib를 사용한 시각화
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.plot(times, x_positions, label="X Position")
    plt.plot(times, y_positions, label="Y Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Robot Position Over Time")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(1, 2, 2)
    plt.plot(x_positions, y_positions, label="Path")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Robot Path (X vs Y)")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    
    # 생성된 플롯을 파일로 저장하거나 화면에 표시
    # plt.savefig("robot_trajectory.png")
    # plt.show() # 주의: plt.show()는 블로킹 함수일 수 있으므로,
    #            # Isaac Sim의 메인 루프와 함께 사용할 때는 주의 필요.
    #            # 비대화형 백엔드를 사용하거나, 별도 스레드에서 실행 고려.
    
    print("Data analysis and visualization example finished.")
    # Isaac Sim 스크립트에서는 Matplotlib 창을 직접 띄우는 것보다
    # 이미지를 파일로 저장하고 외부에서 확인하거나,
    # omni.ui를 사용하여 이미지를 표시하는 것이 더 일반적일 수 있습니다.
    ```

**고려 사항:**

*   **실행 환경:** Isaac Sim의 Python 환경과 외부 라이브러리가 설치된 환경이 일치해야 합니다. 가상 환경 사용을 권장합니다.
*   **성능:** 매우 무거운 계산을 수행하는 라이브러리는 Isaac Sim의 실시간 성능에 영향을 줄 수 있습니다. Python의 멀티스레딩/멀티프로세싱을 사용하거나, 계산이 오래 걸리는 작업은 비동기적으로 처리하는 것을 고려해야 합니다.
*   **UI 통합:** 외부 GUI 라이브러리(Tkinter, PyQt 등)로 만든 UI는 Isaac Sim의 메인 UI 루프와 별도로 실행될 수 있으며, `omni.ui`와 직접 통합되지 않을 수 있습니다. `omni.ui`를 사용하여 확장 기능을 개발하는 것이 UI 통합 측면에서는 더 자연스럽습니다.
*   **블로킹 호출:** `plt.show()`와 같이 프로그램의 실행을 중단시키는(blocking) 함수는 Isaac Sim의 메인 업데이트 루프를 멈추게 할 수 있으므로 주의해서 사용해야 합니다. 비대화형(non-interactive) 모드로 사용하거나, 별도의 스레드/프로세스에서 실행하는 것을 고려하십시오.

## 2. TCP/IP 또는 기타 미들웨어를 사용한 통신

Isaac Sim을 완전히 다른 프로세스나 다른 컴퓨터에서 실행되는 외부 애플리케이션(예: 로봇 제어 시스템, 디지털 트윈 대시보드, AI 학습 환경)과 실시간으로 데이터를 주고받아야 할 경우, 표준 네트워크 통신 프로토콜을 사용할 수 있습니다.

**주요 통신 방식:**

*   **TCP/IP (Transmission Control Protocol/Internet Protocol):**
    *   신뢰성 있는 양방향 연결을 제공하는 표준 네트워크 프로토콜입니다.
    *   Python의 `socket` 모듈을 사용하여 TCP 서버 또는 클라이언트를 Isaac Sim 스크립트 내에 구현할 수 있습니다.
    *   **서버 역할:** Isaac Sim이 외부 클라이언트의 연결을 기다리고 요청을 처리하거나 데이터를 전송합니다.
    *   **클라이언트 역할:** Isaac Sim이 외부 서버에 연결하여 데이터를 요청하거나 전송합니다.
    *   **데이터 형식:** 전송되는 데이터는 바이트 스트림이므로, 구조화된 데이터(예: JSON, XML, Protocol Buffers)를 사용하려면 직렬화/역직렬화 과정이 필요합니다.
*   **UDP (User Datagram Protocol):**
    *   TCP보다 빠르지만 신뢰성이 낮은 비연결형 프로토콜입니다. 데이터 손실이 허용되는 실시간 스트리밍(예: 간단한 원격 측정 데이터)에 적합할 수 있습니다. Python의 `socket` 모듈로 구현 가능합니다.
*   **HTTP (Hypertext Transfer Protocol):**
    *   웹 통신에 널리 사용되는 프로토콜입니다. Isaac Sim이 HTTP 서버(예: Flask, FastAPI 라이브러리 사용) 또는 클라이언트(예: `requests` 라이브러리 사용) 역할을 하여 REST API를 통해 외부 시스템과 상호작용할 수 있습니다.
    *   상태 조회, 간단한 명령 전송 등에 유용합니다.
*   **WebSocket:**
    *   단일 TCP 연결을 통해 양방향 전이중 통신 채널을 제공하는 프로토콜입니다. 실시간 웹 애플리케이션과의 지속적인 데이터 교환에 적합합니다. Python의 `websockets` 라이브러리 등을 사용할 수 있습니다.
*   **메시지 큐 (Message Queues, 예: ZeroMQ, RabbitMQ, Kafka):**
    *   비동기 메시지 기반 통신을 위한 미들웨어입니다. 발행-구독(Publish-Subscribe), 요청-응답(Request-Reply) 등 다양한 통신 패턴을 지원하며, 분산 시스템 간의 느슨한 결합(loose coupling)을 가능하게 합니다.
    *   Isaac Sim과 외부 애플리케이션이 각각 메시지 큐의 생산자(producer) 또는 소비자(consumer)가 되어 데이터를 교환합니다.
    *   Python용 클라이언트 라이브러리를 사용하여 연동합니다. (예: `pyzmq` for ZeroMQ)
*   **ROS/ROS2 (Robot Operating System):**
    *   로봇 애플리케이션 개발을 위한 표준 플랫폼으로, 자체적인 통신 미들웨어(토픽, 서비스, 액션)를 제공합니다. Isaac Sim은 ROS/ROS2 브리지를 통해 ROS 생태계와 긴밀하게 통합될 수 있습니다. (이전 "ROS 2 Humble 연동" 섹션 참조) 이는 로봇 관련 애플리케이션과의 연동에 가장 권장되는 방식 중 하나입니다.

**구현 예시: TCP/IP 소켓 통신 (간단한 서버)**

다음은 Isaac Sim Python 스크립트가 TCP 서버 역할을 하여 외부 클라이언트로부터 간단한 메시지를 받고 응답하는 예시입니다.

```python
import socket
import threading
import omni.kit.app

# 서버 설정
HOST = 'localhost'  # 또는 '0.0.0.0'으로 설정하여 외부에서도 접속 가능하게
PORT = 12345
BUFFER_SIZE = 1024

server_socket = None
server_thread = None
is_server_running = False

def handle_client(conn, addr):
    global is_server_running
    print(f"Connected by {addr}")
    try:
        while is_server_running: # 서버 실행 중일 때만 반복
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break # 클라이언트 연결 종료
            
            message = data.decode('utf-8')
            print(f"Received from client: {message}")
            
            # 예시: 수신된 메시지에 따라 Isaac Sim 내에서 작업 수행
            if message.strip().lower() == "get_time":
                # 현재 시뮬레이션 시간 (예시, 실제 API는 다를 수 있음)
                current_time = omni.timeline.get_timeline_interface().get_current_time() * omni.timeline.get_timeline_interface().get_time_codes_per_seconds()
                response = f"Isaac Sim time: {current_time}"
            elif message.strip().lower() == "quit_server":
                response = "Server shutting down command received."
                # is_server_running = False # 메인 스레드에서 종료하도록 플래그 설정
            else:
                response = f"Echo: {message}"
            
            conn.sendall(response.encode('utf-8'))
    except ConnectionResetError:
        print(f"Client {addr} disconnected.")
    except Exception as e:
        print(f"Error handling client {addr}: {e}")
    finally:
        conn.close()
        print(f"Connection with {addr} closed.")

def server_listen():
    global server_socket, is_server_running
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 주소 재사용 옵션
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen()
        print(f"TCP Server listening on {HOST}:{PORT}")
        is_server_running = True

        while is_server_running:
            try:
                # 비블로킹으로 만들기 위해 타임아웃 설정 또는 select 모듈 사용 고려
                server_socket.settimeout(1.0) # 1초 타임아웃
                conn, addr = server_socket.accept()
                client_thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
                client_thread.start()
            except socket.timeout:
                # 타임아웃 발생 시 is_server_running 플래그 다시 확인
                if not is_server_running:
                    break
                continue # 다시 accept 대기
            except Exception as e:
                if is_server_running: # 서버가 여전히 실행 중이어야 할 때만 오류 로깅
                    print(f"Error accepting connection: {e}")
                break # 루프 종료
                
    except Exception as e:
        print(f"Server socket error: {e}")
    finally:
        if server_socket:
            server_socket.close()
        print("TCP Server has shut down.")

# Isaac Sim 확장 기능의 on_startup에서 호출
def start_server():
    global server_thread, is_server_running
    if server_thread is None or not server_thread.is_alive():
        is_server_running = False # 이전 상태 초기화
        server_thread = threading.Thread(target=server_listen, daemon=True)
        server_thread.start()
        print("TCP Server thread started.")

# Isaac Sim 확장 기능의 on_shutdown에서 호출
def stop_server():
    global is_server_running, server_socket, server_thread
    print("Attempting to stop TCP server...")
    is_server_running = False
    # 소켓을 닫아 accept()에서 블로킹된 스레드가 빠져나오도록 함
    if server_socket:
        # server_socket.shutdown(socket.SHUT_RDWR) # 필요에 따라 사용
        server_socket.close() 
        # server_socket = None # 참조 제거
    if server_thread and server_thread.is_alive():
        server_thread.join(timeout=2.0) # 스레드 종료 대기 (타임아웃 설정)
    print("TCP Server stop sequence finished.")

# 사용 예시 (확장 기능 내에서)
# class MyExtension(omni.ext.IExt):
#     def on_startup(self, ext_id):
#         start_server()
#
#     def on_shutdown(self):
#         stop_server()

# 독립 실행형 스크립트의 경우:
# if __name__ == "__main__":
#     start_server()
#     try:
#         while True: # 메인 스레드는 다른 작업을 하거나 대기
#             # Isaac Sim 업데이트 루프와 함께 사용 시 주의
#             # omni.kit.app.get_app().update() # Kit 앱 루프 실행
#             # time.sleep(0.1)
#             # is_server_running이 False가 되면 루프 탈출 로직 필요
            
#             # 이 예제에서는 간단히 실행하고, 외부에서 종료 명령을 보내거나
#             # Ctrl+C 등으로 종료한다고 가정.
#             # 실제 Isaac Sim 환경에서는 on_shutdown에서 stop_server() 호출이 더 적절.
#             if not is_server_running and server_thread and not server_thread.is_alive():
#                 break
#             pass
#     except KeyboardInterrupt:
#         print("Keyboard interrupt received, shutting down server.")
#     finally:
#         stop_server()

```

**외부 클라이언트 예시 (Python):**

```python
import socket

HOST = 'localhost'
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        message_to_send = input("Enter message for Isaac Sim (or 'quit_server' to stop server, 'exit' to close client): ")
        if message_to_send.lower() == 'exit':
            break
        
        s.sendall(message_to_send.encode('utf-8'))
        data = s.recv(1024)
        print(f"Received from server: {data.decode('utf-8')}")
        
        if message_to_send.lower() == 'quit_server' and "shutting down" in data.decode('utf-8').lower():
            break
print("Client closed.")
```

**고려 사항:**

*   **스레딩 및 비동기 처리:** 네트워크 통신은 블로킹 작업이 될 수 있으므로, Isaac Sim의 메인 스레드(UI 및 시뮬레이션 루프)를 방해하지 않도록 별도의 스레드에서 실행하거나 비동기 I/O를 사용해야 합니다. 위 예제에서는 `threading`을 사용했습니다.
*   **데이터 직렬화:** 복잡한 데이터를 주고받을 때는 JSON, XML, Protocol Buffers, MessagePack 등과 같은 표준 직렬화 방식을 사용하여 데이터 구조를 정의하고 파싱하는 것이 좋습니다.
*   **오류 처리 및 연결 관리:** 네트워크는 불안정할 수 있으므로, 연결 끊김, 타임아웃, 데이터 손실 등의 오류 상황에 대한 견고한 처리 로직이 필요합니다.
*   **보안:** 외부 네트워크에 서비스를 노출할 경우, 인증 및 암호화와 같은 보안 메커니즘을 고려해야 합니다.
*   **방화벽:** Isaac Sim이 실행되는 컴퓨터의 방화벽 설정에서 해당 포트(예: 12345)가 외부 연결을 허용하도록 설정되어야 할 수 있습니다.

외부 애플리케이션과의 연동은 Isaac Sim의 활용 범위를 크게 넓혀, 데이터 기반 시뮬레이션, 디지털 트윈 구축, 복잡한 시스템 통합 등을 가능하게 합니다. 프로젝트의 요구 사항에 가장 적합한 라이브러리 및 통신 방식을 선택하고, 성능과 안정성을 고려하여 신중하게 구현하는 것이 중요합니다.
