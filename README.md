# CODA(Collection of Data for Accidents): 교통사고 현장 보존 로봇 서비스
SLAM 기반 자율주행 로봇 시스템 구성

## 프로젝트 개요

- **목표**: 로봇을 활용해 교통사고 현장의 신속한 대응과 효과적인 후처리를 자동화하여 사회적 비용과 2차 피해를 최소화
- **주요 기능**: 특정 경로 자동 순찰, 사고 감지, 사고 데이터 후처리
- **사용 장비**: TurtleBot4(RPLIDAR A1M8, OAK-D Pro)
- **개발 환경**: Ubuntu 22.04, ROS2 Humble, RTX3060, Colab
- **주요 기술 스택**: ROS2, AutoSLAM, Nav2, OpenCV, YOLO
- **기간**: 2025.05.09 ~ 2025.05.22

## 시연 영상

- 영상 링크: [Demo Video](https://youtube.com/shorts/FaUu1ptY77Y?feature=share)

<div align="center">

[지능 -B 최종 영상.webm](https://github.com/user-attachments/assets/579fdd42-cd57-4035-af2e-b0374ae688f1)

![스크린샷](https://github.com/user-attachments/assets/45af568e-68d3-4b58-9b66-4e2a37ec7447)

</div>

## 시스템 아키텍처

<div align="center">

![시스템 아키텍처](https://github.com/user-attachments/assets/53a3d17e-96e8-4877-a371-c063a2284c0a)

</div>

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
| <1주차> | 2025.05.09(금) ~ 2025.05.15(목) | 지능-1(B) | * 본관 : A-2호 |
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
