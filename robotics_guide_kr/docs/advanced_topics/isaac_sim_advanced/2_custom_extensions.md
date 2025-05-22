# Isaac Sim 고급 토픽: 커스텀 확장 기능 빌드

Isaac Sim은 매우 유연하고 확장 가능한 플랫폼으로 설계되었으며, 그 핵심에는 강력한 확장 기능(Extension) 시스템이 있습니다. 사용자는 이 시스템을 활용하여 Isaac Sim의 기본 기능을 확장하고, 자신만의 도구, UI, 또는 시뮬레이션 로직을 추가할 수 있습니다. OmniGraph가 시뮬레이션 내 로직 흐름을 정의한다면, 확장 기능은 Isaac Sim 애플리케이션 자체의 기능을 보다 근본적으로 확장하는 방법입니다.

이 섹션에서는 Isaac Sim의 확장 시스템에 대한 소개와 함께, 간단한 UI를 가진 확장 기능을 개발하는 단계, 그리고 개발에 필요한 핵심 API 및 유틸리티에 대해 알아봅니다.

## 1. Isaac Sim 확장 시스템 소개

**확장 기능(Extension)이란?**

Isaac Sim(및 Omniverse 플랫폼)의 확장 기능은 특정 기능을 수행하는 독립적인 모듈 또는 플러그인입니다. 각 확장 기능은 Python으로 작성되며, 자체적인 UI 요소, 로직, 그리고 다른 확장 기능과의 상호작용을 포함할 수 있습니다. Isaac Sim의 많은 기본 기능들(예: ROS 브리지, URDF 임포터, 콘텐츠 브라우저)도 실제로는 확장 기능 형태로 구현되어 있습니다.

**확장 시스템의 장점:**

*   **모듈성:** 각 기능이 독립적인 모듈로 분리되어 있어 개발, 테스트, 유지보수가 용이합니다.
*   **유연성 및 확장성:** 사용자는 필요한 기능을 직접 개발하여 Isaac Sim에 추가하거나, 기존 기능을 수정/확장할 수 있습니다.
*   **커뮤니티 및 공유:** Omniverse 플랫폼은 확장 기능을 공유하고 재사용할 수 있는 생태계를 지향합니다.
*   **Python 기반:** Python을 주 언어로 사용하므로 방대한 Python 라이브러리와 생태계를 활용할 수 있습니다.

**확장 기능의 구조 (일반적인 파일 구조):**

하나의 확장 기능은 보통 다음과 같은 디렉토리 및 파일 구조를 가집니다.

```
my_custom_extension_package/  # 확장 기능 패키지 루트 (예: com.example.my_extension)
├── config/
│   └── extension.toml        # 확장 기능 설정 파일 (필수)
├── data/                     # UI 아이콘, 스타일시트 등 정적 데이터
├── docs/                     # 문서
└── my_custom_extension_package/  # 실제 Python 코드가 위치하는 모듈 (패키지 이름과 동일하게)
    ├── __init__.py
    ├── extension.py          # 확장 기능의 메인 로직 (on_startup, on_shutdown 등)
    ├── window.py             # (선택 사항) UI 창 정의
    └── style.py              # (선택 사항) UI 스타일 정의
    └── utils.py              # (선택 사항) 유틸리티 함수
```

*   **`extension.toml` (필수):**
    *   확장 기능의 메타데이터(이름, 버전, 설명, 작성자 등)와 의존성(다른 확장 기능)을 정의하는 TOML 형식의 설정 파일입니다.
    *   확장 기능 관리자가 이 파일을 읽어 확장 기능을 인식하고 로드합니다.
    *   예시:
        ```toml
        [package]
        # 확장 기능의 고유 ID (보통 도메인 역순 + 확장 기능 이름)
        name = "com.example.my_ui_extension" 
        version = "0.1.0"
        title = "My Custom UI Extension"
        description = "A simple example of an Isaac Sim UI extension."
        authors = ["Your Name"]
        
        # 의존하는 다른 확장 기능들
        [[dependencies]]
        # Isaac Sim의 UI 프레임워크 확장 기능
        "omni.ui" = {} 
        # Isaac Sim의 핵심 유틸리티 확장 기능
        "omni.isaac.ui" = {} 

        # 실행될 Python 모듈 지정
        [python.module]
        # name = "my_custom_extension_package" # 패키지 루트 아래 Python 모듈 이름
        ```

*   **`extension.py` (메인 로직):**
    *   `omni.ext.IExt` 인터페이스를 구현하는 Python 클래스를 포함합니다.
    *   `on_startup(self, ext_id: str)`: 확장 기능이 로드될 때 호출되는 메서드. UI 생성, 이벤트 리스너 등록, 초기화 작업 등을 수행합니다.
    *   `on_shutdown(self)`: 확장 기능이 언로드될 때 호출되는 메서드. 리소스 해제, UI 제거 등의 정리 작업을 수행합니다.

## 2. 간단한 UI 확장 기능 개발 단계

간단한 버튼과 레이블을 가진 UI 창을 표시하는 확장 기능을 만들어 보겠습니다.

**단계 1: 확장 기능 디렉토리 및 파일 생성**

위에서 설명한 구조에 따라 디렉토리와 파일을 생성합니다. 패키지 이름은 `com.example.my_ui_extension`으로, Python 모듈 이름은 `my_ui_extension_module` (또는 패키지 이름과 동일하게 `com.example.my_ui_extension`)로 가정합니다.

**단계 2: `extension.toml` 파일 작성**

`my_ui_extension_package/config/extension.toml` 파일을 다음과 같이 작성합니다.

```toml
[package]
name = "com.example.my_ui_extension"
version = "0.1.0"
title = "My Custom UI Window"
description = "A simple UI extension with a button and a label."
authors = ["Your Name"]

# Python 모듈 경로 (Python 코드가 있는 폴더 이름)
# 여기서는 패키지 이름과 동일한 폴더를 사용한다고 가정
python_module_path = "my_ui_extension_module" 

[[dependencies]]
"omni.ui" = {} # omni.ui는 UI 요소를 만드는 데 필요
"omni.kit.window.popup_dialog" = {} # 팝업 다이얼로그 (예시용)

# 실행될 메인 Python 모듈 지정 (폴더 이름.파일 이름 - .py 제외)
[python.module]
name = "my_ui_extension_module.extension" 
```
**주의:** `python_module_path`는 `config` 폴더와 같은 레벨에 있는 Python 코드 폴더를 지정합니다. `[python.module]`의 `name`은 해당 폴더 내에서 `IExt`를 구현한 클래스가 있는 파일 경로를 모듈 형태로 지정합니다.

**단계 3: `my_ui_extension_module/extension.py` 파일 작성**

```python
import omni.ext
import omni.ui as ui
import omni.kit.window.popup_dialog as popup_dialog # 예시용

# 확장 기능의 메인 클래스
class MyExtension(omni.ext.IExt):
    WINDOW_TITLE = "My Custom Window"
    MENU_PATH = f"Window/{WINDOW_TITLE}" # 메뉴에 표시될 경로

    def on_startup(self, ext_id: str):
        print(f"[{ext_id}] MyCustomExtension startup")

        self._window = None # UI 창 객체를 저장할 변수
        
        # 메뉴에 창을 여는 항목 추가
        self._editor_menu = omni.kit.ui.get_editor_menu()
        if self._editor_menu:
            self._menu = self_editor_menu.add_item(
                MyExtension.MENU_PATH, self._build_window, toggle=True, value=True
            )
        
        # 초기 창 생성 및 표시
        self._build_window()

    def _build_window(self):
        if not self._window:
            self._window = ui.Window(
                MyExtension.WINDOW_TITLE, 
                width=300, height=200, 
                visible=True, # 시작 시 보이도록
                dockPreference=ui.DockPreference.FREE # 자유롭게 떠다니는 창
            )
            self._window.set_visibility_changed_fn(self._on_visibility_changed)
            
            # 창 내용 빌드
            with self._window.frame:
                with ui.VStack(spacing=5, height=0): # 수직 정렬 컨테이너
                    ui.Label("Hello from My Custom Extension!", alignment=ui.Alignment.CENTER)
                    
                    def on_button_click():
                        print("Custom button clicked!")
                        popup_dialog.show_popup(
                            title="Information",
                            message="The custom button was clicked.",
                            ok_label="OK"
                        )

                    ui.Button("Click Me", clicked_fn=on_button_click)
        
        # 창 가시성 업데이트
        if self._window:
            self._window.visible = True # 메뉴에서 토글 시 보이도록

    def _on_visibility_changed(self, visible):
        # 메뉴 체크 상태 동기화
        if self._menu:
            omni.kit.ui.get_editor_menu().set_value(MyExtension.MENU_PATH, visible)
        if not visible:
            # 창이 닫힐 때 (메모리에서 완전히 제거하지 않고 숨김 처리)
            # self._window = None # 필요에 따라 창을 완전히 제거하고 싶다면 주석 해제
            pass

    def on_shutdown(self):
        print(f"[{self.ext_id}] MyCustomExtension shutdown")
        if self._menu:
            omni.kit.ui.get_editor_menu().remove_item(self._menu)
            self._menu = None
        if self._window:
            self._window.destroy() # 창 명시적 파괴
            self._window = None
```

**단계 4: 확장 기능 로드 및 테스트**

1.  **확장 기능 검색 경로 추가:**
    *   Isaac Sim을 실행합니다.
    *   상단 메뉴에서 **Window > Extensions**를 선택하여 확장 기능 관리자 창을 엽니다.
    *   창 왼쪽 상단의 톱니바퀴 아이콘(설정)을 클릭하고 "Extension Search Paths" 탭을 선택합니다.
    *   "+" 버튼을 눌러 `my_custom_extension_package` 폴더가 있는 **상위 폴더** (예: `my_custom_extension_package`가 `~/Developer/MyExtensions/my_custom_extension_package`에 있다면, `~/Developer/MyExtensions`를 추가)를 검색 경로에 추가합니다.
2.  **확장 기능 찾기 및 활성화:**
    *   확장 기능 관리자 창에서 검색창에 "My Custom UI Window" (또는 `extension.toml`의 `title`)를 입력합니다.
    *   찾은 확장 기능을 선택하고 토글 스위치를 켜서 활성화합니다.
3.  **결과 확인:**
    *   "My Custom Window"라는 제목의 새 창이 나타나고, 그 안에 레이블과 "Click Me" 버튼이 보여야 합니다.
    *   버튼을 클릭하면 콘솔에 "Custom button clicked!" 메시지가 출력되고 팝업 다이얼로그가 나타나야 합니다.
    *   상단 메뉴의 **Window > My Custom Window** 항목을 통해 창을 켜고 끌 수 있어야 합니다.

## 3. 핵심 API 및 유틸리티

확장 기능 개발 시 자주 사용되는 Omniverse Kit 및 Isaac Sim의 핵심 API와 유틸리티는 다음과 같습니다.

*   **`omni.ext.IExt`:**
    *   모든 확장 기능이 구현해야 하는 기본 인터페이스입니다. `on_startup` 및 `on_shutdown` 메서드를 제공합니다.
*   **`omni.ui` (UI 프레임워크):**
    *   UI 요소를 생성하고 관리하기 위한 핵심 모듈입니다.
    *   `ui.Window`, `ui.Button`, `ui.Label`, `ui.FloatField`, `ui.ComboBox` 등 다양한 위젯 제공.
    *   `ui.VStack`, `ui.HStack`, `ui.ZStack`, `ui.CollapsableFrame` 등 레이아웃 컨테이너 제공.
    *   자세한 사용법은 "Omni UI" 또는 "UI Programming" 관련 공식 문서를 참조하십시오.
*   **`omni.kit.ui.get_editor_menu()`:**
    *   Isaac Sim의 메인 메뉴 바에 접근하여 메뉴 항목을 추가하거나 제거할 수 있게 합니다.
*   **`omni.kit.window.popup_dialog`:**
    *   간단한 정보, 경고, 오류 팝업 다이얼로그를 표시하는 유틸리티입니다.
*   **`omni.usd.get_context()` 및 `pxr.Usd` (USD API):**
    *   USD 스테이지에 접근하고, 프림을 생성/수정/삭제하며, 속성을 읽고 쓰는 등 USD 관련 작업을 수행하기 위한 API입니다.
    *   `usd_context = omni.usd.get_context()`
    *   `stage = usd_context.get_stage()` (현재 USD 스테이지 가져오기)
    *   `prim = stage.GetPrimAtPath("/World/MyRobot")` (특정 경로의 프림 가져오기)
*   **`omni.kit.commands`:**
    *   실행 취소(Undo)/다시 실행(Redo) 기능을 지원하는 명령(Command)을 실행하고 등록하는 시스템입니다. 복잡한 작업을 하나의 명령으로 묶어 관리할 때 유용합니다.
    *   예: `omni.kit.commands.execute('CreatePrim', prim_type='Cube', prim_path='/World/MyCube')`
*   **`omni.isaac.core.utils.*`:**
    *   Isaac Sim 고유의 유틸리티 함수들을 포함합니다 (예: 에셋 경로 가져오기, 로봇 생성, 시뮬레이션 컨텍스트 접근 등).
    *   `omni.isaac.core.utils.prims.define_prim`, `get_prim_at_path`
    *   `omni.isaac.core.utils.stage.add_reference_to_stage`
*   **`omni.kit.app.get_app()`:**
    *   애플리케이션 수준의 이벤트(예: 업데이트 루프, 렌더링 이벤트)에 콜백을 등록하거나, 애플리케이션 설정을 가져오는 등의 작업을 할 수 있습니다.
    *   `update_stream = omni.kit.app.get_app().get_update_event_stream()`
    *   `subscription = update_stream.create_subscription_to_pop(my_update_fn, name="my_update_sub")` (매 프레임 `my_update_fn` 호출)
*   **로깅 (Logging):**
    *   `print()` 대신 `omni.kit.app.get_app().print_message_form("MyExtension", "This is a log message.", omni.kit.app. CARB_LOG_LEVEL_INFO)` 와 같은 로깅 API 사용을 권장합니다. 또는 Python의 표준 `logging` 모듈을 사용할 수도 있습니다.

**팁:**

*   **공식 문서 및 예제 활용:** NVIDIA Omniverse 및 Isaac Sim 개발자 문서에는 확장 기능 개발에 대한 자세한 가이드와 다양한 예제 코드가 포함되어 있습니다. 이를 적극적으로 참조하십시오.
*   **기존 확장 기능 코드 분석:** Isaac Sim에 포함된 많은 기본 확장 기능들의 소스 코드를 살펴보는 것은 좋은 학습 방법입니다. (확장 기능 관리자에서 소스 코드 경로 확인 가능)
*   **작게 시작하여 점진적으로 확장:** 처음에는 간단한 기능부터 시작하여 점차 복잡한 기능을 추가해 나가는 방식으로 개발하는 것이 좋습니다.

커스텀 확장 기능 개발은 Isaac Sim을 자신의 특정 요구 사항에 맞게 최적화하고 자동화하는 강력한 방법입니다. 이를 통해 반복적인 작업을 줄이고, 새로운 도구를 통합하며, 연구 및 개발 생산성을 크게 향상시킬 수 있습니다.
