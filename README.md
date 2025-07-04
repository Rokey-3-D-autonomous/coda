# CODA(Collection of Data for Accidents): 교통사고 현장 보존 로봇 서비스
SLAM 기반 자율주행 로봇 시스템 구성

## 프로젝트 개요

- **목표**: 로봇을 활용해 교통사고 현장의 신속한 대응과 효과적인 후처리를 자동화하여 사회적 비용과 2차 피해를 최소화
- **주요 기능**: 특정 경로 자동 순찰, 사고 감지, 사고 데이터 후처리
- **사용 장비**: TurtleBot4(RPLIDAR A1M8, OAK-D Pro)
- **개발 환경**: Ubuntu 22.04, ROS2 Humble, RTX3060, Colab
- **주요 기술 스택**: ROS2, AutoSLAM, Nav2, OpenCV, YOLO
- **기간**: 2025.05.16 ~ 2025.05.22

## 시연 영상

- 영상 링크: [Demo Video](https://youtube.com/shorts/FaUu1ptY77Y?feature=share)

<div align="center">

[지능 -B 최종 영상.webm](https://github.com/user-attachments/assets/579fdd42-cd57-4035-af2e-b0374ae688f1)

<!--
![스크린샷](https://github.com/user-attachments/assets/45af568e-68d3-4b58-9b66-4e2a37ec7447)
-->

</div>

## 시스템 아키텍처

<div align="center">

![시스템 아키텍처](https://github.com/user-attachments/assets/53a3d17e-96e8-4877-a371-c063a2284c0a)

</div>

## 상세 설명

### 문제정의

- 교통사고로 인한 사회적 비용이 지속적으로 증가하고 있음(2022년 국가예산의 4.3%).
- 일반 운전자 촬영 사진의 법적·보험적 활용 한계.
- 고속도로 2차 사고의 치사율이 일반 사고보다 7배 이상 높음.
- 신속하지 못한 현장 대응, 비전문가에 의한 부정확한 정보 기록이 2차 피해와 사회적 부담을 증가시킴.

### 해결방안

- TurtleBot 기반 자율주행 로봇 시스템과 서버를 연동.
- 사고 감지, 현장 자동 촬영, 차량 통제, 데이터 실시간 전송을 자동화.
- AI 기반(객체인식, 위치 추정) 사고 감지와 SLAM·Navigation을 통한 정확한 자율 순찰.
- 촬영된 데이터를 바탕으로 자동 보고서(HTML) 생성까지 전 과정 자동화.

### 주요기능

- **순찰 및 사고 감지:** 순찰 경로 내 자율주행, YOLO 기반 객체 인식으로 사고 실시간 감지.
- **사고 현장 자동 촬영:** 사고 감지 시 촬영봇이 현장 이동 후 사진·3D 포인트클라우드 획득.
- **좌표 변환 및 데이터 전송:** 카메라 좌표를 글로벌 좌표로 변환하여 서버에 보고.
- **차량 통제:** 사고 감지 시 경고음 및 안내화면, 통제 종료 후 순찰 재개.
- **자동 리포트 생성:** 사고 기록, 촬영 데이터, 위치 정보로 HTML 보고서 자동화.
- **통합 서버 및 모듈화:** 노드별 통신 구조 설계, 각 단계의 상태 관리(FSM), 실시간 명령 처리 및 테스트 자동화.
- **시스템 신뢰성 강화:** ROS2 QOS, 네트워크 신뢰성 확보, 순찰-감지-촬영-복귀까지 전 과정 완전 자동화

## 프로젝트 기여자

- 김승주: lunaticju@gmail.com
- 김재권: kimjg1219@gmail.com
- 이호준: hojun7889@gmail.com
- 위석환: llaayy.kr@gmail.com
- 지예은: jyebubu@gmail.com
- 최초인: choinchoi0331@gmail.com
- 홍지원: jw1122.h@gmail.com

## 교육과정 및 참고자료

### 교육과정

<div align="center">

| 주차 | 기간 | 구분 | 강의실 |
| --- | --- | --- | --- |
| <2주차> | 2025.05.16(금) ~ 2025.05.22(목) | 지능-1(B) | * 본관 : A-2호 |

| **차시** | **구분** | **세부사항** | **평가** | **팀구성** |
| --- | --- | --- | --- | --- |
| 1 | 프로젝트 계획 및 환경 구축 | 시스템 개발 프로세스의 이해, 개발 환경 구축 | O | 4인 1팀 |
| 2 | 기술 탐색 및 검증 | AI VISION 기술 탐색 및 검증_Object Detection |  | 4인 1팀 |
| 3 | 기술 탐색 및 검증 | AI VISION 기술 탐색 및 검증_Depth 활용 | O | 4인 1팀 |
| 4 | 기술 탐색 및 검증 | AMR 제어 기술 탐색 및 검증_SLAM & Navigation |  | 4인 1팀 |
| 5 | 기술 탐색 및 검증 | AMR 제어 기술 탐색 및 검증_통합 Multi-Robot | O | 4인 1팀 |
| 6 | 프로젝트 설계 | 시스템 설계 및 환경 구성 |  | 2개 팀  |
| 7 | 개발 | 기능 구현 및 Unit Test |  | 2개 팀 |
| 8 | 개발 | 기능 구현 및 Unit Test |  | 2개 팀 |
| 9 | 개발 | 통합 시스템 구축 및 테스트 |  | 2개 팀 |
| 10 | 프로젝트 발표 | 프로젝트 발표 및 시연, 산출물 정리, 기술 컨퍼런스 | O | 2개 팀 |

</div>

### 참고자료

- https://github.com/turtlebot/turtlebot4
- https://docs.luxonis.com/
