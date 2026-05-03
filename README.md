# 🤖 Bluetooth Setting Serving Robot System

> AVR MCU 기반 블루투스 제어 서빙 로봇 시스템  
> Android 애플리케이션을 통해 로봇 이동 경로를 설정하고, EEPROM에 저장된 테이블 경로를 기반으로 자동 서빙 및 복귀 기능을 수행하는 임베디드 프로젝트

---

## 📌 Project Overview

본 프로젝트는 음식점 및 카페 환경에서 활용 가능한 **테이블 서빙 자동화 로봇 시스템**을 목표로 개발되었습니다.  
사용자는 Android 블루투스 애플리케이션을 이용하여 로봇의 이동 경로를 직접 설정할 수 있으며,  
설정된 경로 데이터는 AVR 내부 EEPROM에 저장되어 버튼 입력만으로 해당 테이블까지 자동 이동 및 복귀가 가능합니다.

단순 수동 조종이 아닌,

- **경로 학습(Path Learning)**
- **이동 거리 저장(Encoder Count)**
- **방향 전환 기록(Direction Stack)**
- **테이블 자동 서빙(Auto Serving)**
- **복귀 시스템(Return Mode)**

까지 포함한 임베디드 자동화 제어 시스템입니다.

---

## 🎥 Demonstration Video

▶ [서빙 로봇 동작 영상 보기](https://www.youtube.com/watch?v=5yJZ0IHTp8U)

---

## 🔗 GitHub Repository

### 📱 Android Bluetooth Controller App
https://github.com/CTMasdf/OSS_team_serveware_serving_robot_bluetooth_setting

### 🤖 AVR Serving Robot Embedded System
https://github.com/CTMasdf/OSS_team_serveware_serving_robot_2

---

## 🛠 Development Environment

| Category | Contents |
|----------|----------|
| MCU | ATmega128 |
| Language | Embedded C, Java(Android) |
| IDE | AVR Studio / Android Studio |
| Communication | Bluetooth UART Serial |
| Storage | Internal EEPROM |
| Motor Control | DC Motor + Encoder |
| Display | FND + LED Indicator |
| Mobile App | Android Bluetooth Controller |

---

## ⚙ System Architecture

```text
[ Android Bluetooth App ]
          ↓ Bluetooth UART Command
[ HC-06 Bluetooth Module ]
          ↓ Serial Communication
[ ATmega128 Main Controller ]
 ├── DC Motor Control
 ├── Encoder Distance Measurement
 ├── EEPROM Path Save/Load
 ├── FND Table Display
 ├── Push Button Event
 └── Auto Serving Logic
```

---

## ✨ Core Features

### 1. 블루투스 기반 수동 이동 제어
Android 앱에서 전진 / 후진 / 좌회전 / 우회전 버튼 입력 시  
Bluetooth UART 통신을 통해 실시간으로 로봇 구동 제어가 가능합니다.

---

### 2. 테이블 경로 설정 모드 (Path Setting Mode)
사용자가 특정 테이블 번호를 선택한 후 로봇을 직접 이동시키며  
각 이동 거리(엔코더 값)와 방향 전환 데이터를 실시간 기록합니다.

기록 데이터:

- `run_data[]` : 이동 거리 저장
- `move_data[]` : 방향 저장

---

### 3. EEPROM 기반 경로 저장 기능
설정 완료 시 테이블별 이동 데이터를 AVR 내부 EEPROM에 저장하여  
전원이 꺼져도 경로 정보가 유지됩니다.

저장 정보:

- 테이블 번호
- 이동 거리 데이터
- 방향 전환 데이터

---

### 4. 버튼 입력 자동 서빙 모드
사용자가 FND를 통해 테이블 번호를 선택 후 버튼을 누르면  
EEPROM에 저장된 해당 경로를 읽어 자동으로 서빙 동작을 수행합니다.

자동 수행 항목:

- 테이블까지 자율 이동
- 도착 후 대기
- 물건 전달 완료 버튼 입력 대기
- 출발지 자동 복귀

---

### 5. FND 및 LED 상태 인터페이스
로봇의 현재 상태를 사용자에게 직관적으로 표시합니다.

- 현재 선택 테이블 번호 출력
- 설정 완료 여부 점멸 표시
- 에러 발생 시 `Er` 표시
- 서빙/복귀 상태 LED 표시

---

## 📱 Android Controller Application

안드로이드 앱에서는 다음 기능을 제공합니다.

- 블루투스 ON/OFF
- HC-06 연결
- 설정모드 진입 / 취소
- 수동 방향 제어
- 테이블 도착 신호 입력
- UART 데이터 송수신 상태 확인

### 송신 명령 체계

| Command | Function |
|---------|----------|
| `11` | 설정모드 진입 |
| `00` | 설정모드 취소 |
| `==` | 전진 |
| `++` | 후진 |
| `<<` | 좌회전 |
| `>>` | 우회전 |
| `55` | 정지 |
| `22` | 테이블 도착 완료 |

---

## 🤖 Embedded Control Logic

ATmega128 메인 펌웨어는 Timer Interrupt 기반으로 다음 기능을 동시 수행합니다.

- 버튼 입력 처리
- 엔코더 값 측정
- FND 출력 제어
- LED 상태 제어
- DC 모터 제어
- EEPROM 읽기/쓰기
- 자동 서빙 상태머신 처리

### 주요 함수 구성

| Function | Description |
|----------|-------------|
| `button_control()` | 버튼 이벤트 처리 |
| `encoder()` | 엔코더 거리 측정 |
| `FND_LED()` | FND/LED 상태 출력 |
| `DC_MOTOR()` | DC모터 구동 제어 |
| `mode_setting_move()` | 설정모드/서빙모드 로직 |
| `EEPROM_write()` | 경로 저장 |
| `EEPROM_read()` | 경로 불러오기 |

---

## 🔥 Key Technical Achievements

- AVR MCU 기반 실시간 인터럽트 제어 구현
- Android Bluetooth UART 무선 제어 시스템 구축
- EEPROM 비휘발성 경로 저장 로직 설계
- 엔코더 기반 거리 측정 및 이동 재현
- 자동 서빙 → 도착 → 복귀 상태 머신 구현
- FND + LED 사용자 인터페이스 설계

---

## 💡 Expected Effect

본 시스템은 반복적인 서빙 업무를 자동화하여  
소규모 음식점, 카페, 실습형 서비스 로봇 분야에 적용 가능성이 있으며,

향후에는

- 장애물 회피 센서
- 자율주행 알고리즘
- 다중 테이블 맵핑

기능으로 확장 가능한 임베디드 서비스 로봇 플랫폼입니다.

---

## 📷 Project Result

- Bluetooth 수동 제어 가능
- 경로 설정 저장 가능
- 테이블 자동 이동 가능
- 목적지 도착 후 복귀 가능
- EEPROM 데이터 유지 확인 완료

---

## 👨‍💻 Developer

**최태민**  
Embedded System / MCU Control / Android Bluetooth App / EEPROM Logic / Robot Motion Control

---
