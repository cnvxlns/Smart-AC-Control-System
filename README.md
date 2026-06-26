# SmartAC — uC/OS-III 기반 스마트 에어컨 제어 펌웨어

기능 단위 모듈로 분리된 스마트 에어컨 컨트롤러입니다. DHT22 온습도 센서와 리드(창문) 스위치, 버튼 입력을 받아 냉방 상태 머신을 돌리고, 결과를 온보드 LED·외부 부저·TM1637 4-digit 7-세그먼트에 표시하며, UART로 진단 로그를 출력합니다.

- **타깃 보드**: NUCLEO-F429ZI (STM32F429ZI, Cortex-M4F)
- **RTOS**: uC/OS-III
- **드라이버**: STM32 StdPeriph
- **툴체인**: TrueSTUDIO(GCC, 기본) / IAR / Keil MDK

> 이 프로젝트는 Micrium의 **STM32F429II-SK** 예제를 NUCLEO-F429ZI로 이식한 것입니다. BSP·포트는 SK 보드 기준이라 몇 가지 보드 차이 대응이 필요합니다([주의사항](#주의사항--known-issues) 참고).

---

## 핀 맵 (NUCLEO-F429ZI)

| 기능 | Arduino 핀 | STM32 핀 | 비고 |
|---|---|---|---|
| DHT22 DATA | A2 | PC3 | GPIO, **3.3V 풀업 권장** (4.7k~10k) |
| TM1637 CLK | D2 | PF15 | push-pull 출력 |
| TM1637 DIO | D3 | PE13 | open-drain (양방향, ACK 수신) |
| 리드 스위치(창문) | A0 | PA3 | EXTI3, 열림 = HIGH |
| 설정온도 UP 버튼 | A1 | PC0 | EXTI0, 눌림 = LOW (active-low + 풀업) |
| 설정온도 DOWN 버튼 | D8 | PF12 | EXTI12, 눌림 = LOW (active-low + 풀업) |
| USER/B1 버튼 | — | PC13 | 온보드, EXTI13, 눌림 = HIGH (active-high + 풀다운) |
| LD1 (냉방, green) | — | PB0 | 온보드 LED |
| LD2 (수동, blue) | — | PB7 | 온보드 LED |
| LD3 (경고, red) | — | PB14 | 온보드 LED |
| 외부 부저 | D9 | PD15 | **active-low** 출력 |
| UART 디버그 | — | PD8/PD9 | USART3, 115200 8N1 (ST-Link VCP) |
| 마이크로초 타이머 | — | TIM2 | free-running 1 MHz |

> 외부 배선은 A0~A5 / D0~D15 Zio 헤더만 사용합니다. DHT22(PC3)는 ADC/EXTI가 아닌 **순수 GPIO**로 씁니다.

### TM1637 / DHT22 회로 메모
- **TM1637 VCC는 반드시 3.3V**로 공급하세요. 5V로 주면 3.3V open-drain HIGH가 모듈 입력 임계값에 못 미치거나 모듈 풀업이 STM32 핀에 5V를 인가해 깨짐/손상이 발생합니다.
- TM1637 GND와 NUCLEO GND 공통, DIO는 가급적 3.3V로 외부 풀업(4.7k~10k) 추가 시 가장 안정적입니다.
- DHT22도 3.3V 공급 + DATA 풀업 권장. (Arduino에서 쓰던 5V 풀업을 그대로 쓰지 마세요.)

---

## 빌드 & 플래시

### TrueSTUDIO (기본)
1. `TrueSTUDIO/` 프로젝트(`OS3`)를 import
2. **Project → Clean → Build** (설정 변경 후엔 반드시 clean)
3. ST-Link로 플래시 / 디버그 (`OS3.elf.launch`)

### 부동소수점 설정 — 중요
이 프로젝트는 **소프트웨어 부동소수점(`-mfloat-abi=soft`)** 으로 빌드해야 합니다.

- uC/OS-III 포트가 `Ports/ARM-Cortex-M4/Generic`(**non-FPU**)라서 문맥전환 시 FPU 레지스터를 보존하지 않습니다.
- 하드웨어 FPU(`hard`/`softfp`)로 빌드하면 태스크의 float 연산이 문맥전환과 얽혀 **INVSTATE HardFault**(간헐적, 부팅 몇 초 뒤 사망)가 발생합니다.
- TrueSTUDIO: *Properties → C/C++ Build → Settings → Target → Floating point = "Software implementation"* (모든 툴에 적용). `.cproject`에는 `...target.fpu.soft`로 설정돼 있습니다.

### 시리얼 모니터
115200 8N1로 ST-Link VCP를 열면 부팅 시 `SmartAC started`, 이후 1초마다 `INPUT ...` 진단 줄이 출력됩니다.

---

## 아키텍처

### 태스크 구성 (uC/OS-III)

| 태스크 | 우선순위 | 주기 | 역할 |
|---|---|---|---|
| `MonitorTask` | 4 | 1000 ms | UART로 입력/센서/진단 로그 출력 |
| `SensorTask` | 5 | 100 ms (DHT는 2500 ms) | DHT22 읽기, 리드 스위치 감지 → `SensorQ` |
| `UserInputTask` | 6 | 10 ms | 버튼 폴링·디바운스 → `UserEventFlags` |
| `ControlTask` | 7 | 이벤트 구동 | 상태 머신 실행 → `DisplayQ`, `ActuatorQ` |
| `DisplayTask` | 8 | 100 ms | TM1637 4-digit 표시 |
| `ActuatorTask` | 9 | 25 ms | LED·부저 구동 |

- **큐**: `SensorQ`, `DisplayQ`, `ActuatorQ` (depth 8). 메시지 본문은 정적 슬롯 링버퍼에 보관(동적 할당 없음).
- **플래그 그룹**: `UserEventFlags` (UP/DOWN/MANUAL/POWER/BUZZER_CLICK).
- **뮤텍스**: `UartMutex` (UART 출력 직렬화).
- 태스크 스택: 각 1024 words, `OS_OPT_TASK_STK_CHK`로 사용량 추적(`sensor_stk_free` 로그).

### 데이터 흐름
```
SensorTask ──SensorQ──┐
                      ├─► ControlTask ──DisplayQ──► DisplayTask ─► TM1637
UserInputTask ─Flags──┘             └─ActuatorQ──► ActuatorTask ─► LED/Buzzer
```

### 제어 상태 머신 (`Control_Step`)
순수 함수로 구현되어 호스트 테스트(`APP_HOST_TEST`)가 가능합니다.

| 상태 | 의미 |
|---|---|
| `AC_STATE_OFF` | 전원 꺼짐 |
| `AC_STATE_IDLE` | 켜짐, 냉방 불필요 |
| `AC_STATE_COOLING` | 냉방 동작 (레벨 1~3, 온도-설정 차로 결정) |
| `AC_STATE_WINDOW_SUSPECT` | 창문 열림 감지, 일시정지 판단 중 |
| `AC_STATE_AC_PAUSED` | 창문 5초 이상 개방 또는 습도 10%↑ 급등으로 냉방 정지 |
| `AC_STATE_RECOVERY_WAIT` | 창문 닫힘 후 5초 복구 대기 |
| `AC_STATE_MANUAL_OVERRIDE` | 수동 모드 |

- 설정온도: 16~30°C, 기본 24°C
- 버튼: UP/DOWN = 설정온도 ±1, B1 짧게 = 수동 토글, B1 길게(≥1500 ms) = 전원 토글, B1 누름 = 부저 클릭 피드백

---

## 표시 (TM1637 4-digit)

| 상태 | 표시 | 예시 |
|---|---|---|
| OFF | `OFF ` ↔ blank 점멸(1 Hz) | `OFF ` |
| IDLE | 현재온도 + 설정온도 (구분 dp/콜론) | `24.23` |
| COOLING | 현재온도 + 냉방레벨 | `24L2` |
| WINDOW_SUSPECT | `OP` + 개방 경과초 | `OP03` |
| AC_PAUSED | 정지 코드 | `PAUS` |
| RECOVERY_WAIT | `ES` + 남은초(카운트다운) | `ES04` |
| MANUAL_OVERRIDE | 현재온도 + 수동레벨 | `24A2` |
| 센서 무효 | 오류 표시 | `----` / `--L2` |

- 경과초/남은초는 `DisplayTask`가 상태 진입 시각을 자체 추적(방식 A)해 계산하므로 제어 FSM은 건드리지 않습니다.
- 시계형 TM1637(가운데 콜론만 있는 모듈)에서는 IDLE 구분자가 `24:23`으로 보일 수 있습니다(동작 동일).

---

## 입력 처리 / 디바운싱

- **UP/DOWN**: `UserInputTask`가 10 ms마다 폴링하고, **연속 5회(=50 ms) 같은 레벨일 때만** 상태 변경을 확정하는 카운터형 SW 디바운서(`DebounceButton`)를 통과시킵니다. 1회 물리 누름 = 정확히 1 이벤트, 채터링 무시.
- **B1(PC13)**: NUCLEO-144는 active-high라 내부 풀다운 + `APP_B1_PRESSED_LEVEL=SET`로 별도 처리. 짧게/길게 누름을 시간으로 구분.
- EXTI는 활성 상태지만 UP/DOWN 이벤트 판정은 폴링 디바운스로 처리합니다.

---

## DHT22 드라이버

- TIM2(1 MHz) 기반 마이크로초 타이밍으로 LOW/HIGH 펄스 폭을 직접 측정(`high_us > 임계`이면 1).
- 40비트 수신 timing-critical 구간은 `__disable_irq()`로 보호하며, 단계별 실패 코드(`DhtErrorCode`)와 진행 단계(`DhtStage`)를 진단용으로 기록.
- 읽기 주기 2500 ms(사양상 과빈도 읽기 방지).

---

## 시리얼 진단 출력

매초 출력되는 `INPUT` 줄 예시:
```
INPUT button_up=0 button_down=0 button_b1=0 reed_open=1 dht22=1 temp_c=26.4 humidity_pct=65.9 \
dht_fail_count=0 dht_err=0 dht_stage=8 resp_low_us=78 resp_high_us=78 bit_low_us=49 bit_high_us=25 \
us_timer_ready=1 us_ticks_100ms=100000 sensor_stk_free=953 tm1637_ack=7 raw=2,147,1,8,158
```

주요 필드:
- `dht_err` — DHT 실패 분류(1=타이머, 2=response LOW, 3=response HIGH, 4/5=data bit, 6=checksum, 0=정상)
- `us_ticks_100ms` — TIM2 실속도 self-test. **~100000이면 1 MHz 정상**.
- `tm1637_ack` — 마지막 갱신에서 TM1637이 ACK한 바이트 수. **7이면 통신 정상**, 0이면 배선/전원/레벨 문제.
- `sensor_stk_free` — SensorTask 잔여 스택(words). 0에 가까워지면 오버플로 위험.

### HardFault 핸들러
`App_Fault_ISR`가 폴트 시 CFSR/HFSR/BFAR/MMFAR과 `dht_stage`를 UART로 덤프합니다(무한루프 대신). 폴트가 “조용한 멈춤”이 아니라 진단 가능한 형태로 드러납니다.

---

## 진단용 컴파일 스위치 (`app_config.h`)

| 매크로 | 기본값 | 설명 |
|---|---|---|
| `APP_DHT_READ_ENABLE` | 1 | 0이면 DhtRead 생략(RTOS/UART 독립 동작 확인용) |
| `APP_DHT_DEBUG_LOG` | 1 | `DHT before/ok/fail` 로그 |
| `APP_DHT_DISABLE_IRQ` | 1 | 0이면 IRQ 켠 채 읽기(폴트 위치 추적용) |
| `APP_FAULT_DUMP` | 1 | HardFault UART 덤프 |
| `APP_TM1637_DELAY_US` | 50 | TM1637 비트 주기(DIO 상승시간 마진) |

---

## 주의사항 / Known Issues

이식 과정에서 해결한/유의할 보드 차이:

1. **소프트웨어 float 필수** — non-FPU 포트 + 하드웨어 FPU 조합은 간헐적 INVSTATE HardFault. (위 [빌드 설정](#부동소수점-설정--중요) 참고)
2. **FPU 활성화** — `Board_EnableFpu()`가 CPACR로 FPU를 켭니다(이 BSP의 Reset_Handler는 `SystemInit` 호출이 비활성이라). soft-float에서는 무해하며, 혹시 hard-float로 남았을 때의 NOCP 폴트를 막는 보호막 역할.
3. **TM1637 전원 3.3V** + **CLK push-pull** + **비트주기 50 µs** — 약한 내부 풀업/느린 엣지로 인한 데이터 깨짐 방지.
4. **B1 극성** — NUCLEO-144는 active-high(풀다운). NUCLEO-64 계열과 반대.

---

## 파일 구성

| 파일 | 내용 |
|---|---|
| `app.c` | 애플리케이션 시작점(`main`, `App_Start`) |
| `app_config.h` | 타이밍, 핀맵, 태스크, 큐, 진단 매크로 |
| `app_messages.h` | 태스크 간 메시지와 사용자 이벤트 타입 |
| `app_control.c/.h` | 순수 상태 머신과 냉방 제어 로직 |
| `app_board.c/.h` | STM32 GPIO/UART/TIM/EXTI 초기화와 보드 헬퍼 |
| `app_dht22.c/.h` | DHT22 프로토콜과 센서 진단값 |
| `app_tm1637.c/.h` | TM1637 표시 프로토콜과 렌더링 |
| `app_actuator.c/.h` | LED와 부저 출력 |
| `app_monitor.c/.h` | UART 진단 로그 출력 |
| `app_tasks.c/.h` | uC/OS-III 태스크, 큐, 플래그, 런타임 카운터 |
| `app_fault.c/.h` | HardFault 진단 덤프 |
| `app_origin.c` | 이식 전 원본 참고본 |
| `app_cfg.h`, `os_cfg*.h`, `cpu_cfg.h`, `lib_cfg.h` | uC/OS-III · CPU · LIB 설정 |
| `os_app_hooks.c/.h` | OS 훅 |
| `includes.h` | 공통 인클루드 |
| `TrueSTUDIO/`, `IAR/`, `KeilMDK/` | IDE별 프로젝트 |
| `SmartAC_changes_summary.md` | 변경 이력 메모 |
