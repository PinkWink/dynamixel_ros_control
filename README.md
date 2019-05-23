# dynamixel_ros_control


# gear ratio

wheel
1:2.2   모터 한바퀴:바퀴 2.2바퀴

몸통yaw
2.5:1   모터 2.5바퀴:뫁통 한바퀴

머리pan
4:3     모터 4바퀴: 팬 3바퀴?

엘레베이션
풀리크기 55mm
원호길이 pi * 55
모터는 회전각도 출력
풀리크기는 1:1

각도(rad) * 55mm / 2.0



모터4
Velocity I Gain (524): 2209
Velocity P Gain (526): 6153

모터9
Velocity I Gain (586): 14
Velocity P Gain (588): 399



Homing (엘레베이션 축) 방법

1. 리부트
1. 모터를 속도 제어 모드로 변경
2. 토크온
3. 프로파일전류값을 2000으로 설정
4. 100 속도로 이동
5. Moving 상태 체크
6. Moving == 0 이면 모터 속도 0으로 설정
7. 리부트
8. 모터를 확장위치제어 모드로 변경
9. 현재 위치 획득
10. 0으로 이동


