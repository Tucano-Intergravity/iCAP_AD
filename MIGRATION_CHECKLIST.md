# MIGRATION_CHECKLIST

이 체크리스트는 `iCAP_IRIDIUMTEST`를 기준으로 새 STM32 프로젝트를 만들 때 동일 세팅을 재현하기 위한 문서입니다.

## 1) 생성 전 준비

- [ ] 기준 `.ioc` 파일 확인: `iCAP_IRIDIUMTEST.ioc`
- [ ] 대상 MCU/패키지 동일 여부 확인: `STM32H755ZITx`
- [ ] STM32CubeMX standalone 사용 (CubeIDE 2.x 통합 워크플로우 기준)
- [ ] 새 프로젝트는 반드시 새 폴더에 생성 (기존 폴더 덮어쓰기 금지)

## 2) CubeMX에서 새 프로젝트 만들기

- [ ] 기준 `.ioc` 로드
- [ ] `File > Save Project As...`로 새 `.ioc` 이름/경로 저장
- [ ] `Project Manager > Project Name` 확인
- [ ] `Toolchain / IDE = STM32CubeIDE` 설정
- [ ] 코드 생성(Generate Code)

## 3) .ioc 핵심 항목 검증

- [ ] 듀얼코어 프로젝트 구조 유지 (`CM7`, `CM4`)
- [ ] `UART7.BaudRate=19200`
- [ ] UART7 핀: `PF6=UART7_RX`, `PF7=UART7_TX`
- [ ] GPIO 출력 핀: `PF8`, `PF9`, `PF10`, `PC0`, `PC1`, `PC2_C`, `PC3_C`
- [ ] UART7 IRQ 활성화 (`UART7_IRQn`)
- [ ] 사용자 코드 보존 옵션 유지 (`ProjectManager.KeepUserCode=true`)

## 4) 코드 이식

- [ ] 사용자 로직은 생성코드 바깥 사용자 파일 또는 `USER CODE` 블록 중심으로 이식
- [ ] CM7 로직 우선 반영
  - [ ] GPIO power-up 시퀀스
  - [ ] UART7 인터럽트 수신/송신
  - [ ] `AT\r` 주기 송신(1초)
  - [ ] 수신 버퍼에서 `OK/ERROR` 패턴 검사
- [ ] CM4 부트 동기화 코드(HSEM) 유지 여부 확인

## 5) 빌드/실행 검증

- [ ] `CM7` 빌드 성공 (`.elf` 생성)
- [ ] UART7 TX 파형 또는 로그에서 `AT` 송신 확인
- [ ] 응답 프레임(`\r\nOK\r\n` 또는 `\r\nERROR\r\n`) 수신 확인
- [ ] 하드웨어 핀 제어 상태 확인 (`PF8~10`, `PC0~3`)

## 6) 자주 발생하는 문제

- [ ] `Project Name`이 안 바뀌면 `Save Project As...` 먼저 수행
- [ ] 그래도 고정되면 `.ioc` 텍스트에서 아래 2개 직접 수정
  - [ ] `ProjectManager.ProjectName=...`
  - [ ] `ProjectManager.ProjectFileName=....ioc`
- [ ] `.ioc` 파일 읽기 전용(RO) 속성 해제

## 7) Git 권장

- [ ] 템플릿 태그 생성 (`template-h755-iridium-v1` 등)
- [ ] 빌드 산출물(`Debug/`, `CM7/Debug/`) 커밋 제외
- [ ] 문서(`MIGRATION_CHECKLIST.md`, `ARCHITECTURE.md`) 함께 버전 관리

