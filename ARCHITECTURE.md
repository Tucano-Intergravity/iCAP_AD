# ARCHITECTURE

## 프로젝트 목적

`iCAP_IRIDIUMTEST`는 STM32H755 듀얼코어 보드에서 Iridium 모듈 통신을 검증하기 위한 테스트 프로젝트입니다.

- 코어 역할
  - `CM7`: 메인 애플리케이션 실행 (GPIO 제어 + UART7 AT 테스트)
  - `CM4`: 부트 동기화 이후 유휴 루프
- 통신 방식
  - `UART7` 인터럽트 기반 TX/RX
- 테스트 동작
  - 주기적으로 `AT\r` 송신 후 `OK/ERROR` 응답 판정

## 디렉토리 구조

- `CM7/`
  - `Core/Src/main.c`: 핵심 테스트 로직
  - `Core/Src/stm32h7xx_hal_msp.c`: UART7 핀/IRQ MSP 초기화
- `CM4/`
  - `Core/Src/main.c`: HSEM 기반 부트 동기화 + 유휴 루프
- `Common/`
  - 듀얼코어 부트 관련 공통 시스템 코드
- `Drivers/`
  - CMSIS/HAL 드라이버
- `iCAP_IRIDIUMTEST.ioc`
  - CubeMX 설정 소스(핀/클럭/주변장치/코어 컨텍스트)

## 부트 시퀀스(요약)

1. 전원 인가 후 CM7/CM4 시작
2. 듀얼코어 부트 동기화(`HSEM`) 수행
3. CM7이 시스템 초기화 완료 후 CM4 깨움
4. CM7 주변장치 초기화(UART7/GPIO)
5. 앱 로직 시작

## 주변장치/핀 구성(핵심)

- UART7
  - `PF6`: `UART7_RX`
  - `PF7`: `UART7_TX`
  - Baudrate: `19200`
  - IRQ: `UART7_IRQn` 사용
- GPIO Output
  - `PF8`, `PF9`, `PF10`
  - `PC0`, `PC1`, `PC2_C`, `PC3_C`

## 앱 로직 흐름(CM7)

1. GPIO 출력 핀 HIGH 시퀀스 실행 (모듈 전원/제어 의도)
2. UART7 인터럽트 수신 시작
3. `AT\r` 1회 송신
4. 메인 루프에서 1초 주기로 재송신
5. 수신 버퍼에 `\r\nOK\r\n` 또는 `\r\nERROR\r\n` 포함 여부 확인
6. UART 에러 발생 시 버퍼 초기화 후 수신 재시작

## 코드 관리 원칙(재사용용)

- CubeMX 생성영역은 가능한 건드리지 않기
- 사용자 로직은 아래 중 하나로 고립
  - `USER CODE BEGIN/END` 블록
  - 별도 사용자 파일(`App/*.c`, `App/*.h`)로 분리
- `.ioc`는 설정의 단일 진실 공급원으로 취급

## 새 프로젝트 이식 전략

1. 기준 `.ioc`를 로드 후 `Save Project As...`
2. 새 이름/경로로 코드 재생성
3. 사용자 로직 파일 이식
4. 핀/클럭/UART7/듀얼코어 동기화 항목 검증
5. UART AT 응답까지 확인 후 기준 태그 생성

