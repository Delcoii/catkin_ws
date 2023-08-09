성훈이가 언수에게

y = ax3 + bx2 + cx + d 인 curve에 대해
a, b, c, d(코드에선 a3, a2, a1, a0)를 추측하는 코드임.

todo list
1. include/waypoint_save/에 get_lat_lon.cpp파일에서 lane4th.csv 저장 경로 수정해야함

2. armadillo 라이브러리 잘 깔아야함

3. pure pursuit, stanley method,
아울러 이 curve fitting visualize 프로젝트에 필요한
차량에 front_wheel_tf, rear_wheel_tf 생성 코드도 같이 catkin_make해야함

프로젝트 이름은 wheel_center_tf 임

이 프로젝트는 차량 odometry를 기준으로 하여,
/carla/ego_vehicle/vehicle_info 에서 나온 ego_vehicle pose 기준 전륜, 후륜의 위치(x값을 반영)를 tf로 만듬

ego_vehicle을 기준으로 한 front_wheel_tf, rear_wheel_tf를 broadcast한 뒤
map에 대해서 front_wheel_tf, rear_wheel_tf를 listen한 후
그 값을 geometry_msgs::PoseStamped 메세지로 2개의 topic(/front_wheel_pose, /rear_wheel_pose)
를 publish함

해당 프로젝트가 동시에 돌아가는 carla_ros_bridge노드 런치파일을 같이 올려둘 예정
- 이에 따른 objects.json파일도 같이 올릴 예정이니, 런치파일에서 경로 잘 수정해주길 바람

이후 broadcast, listen하는 과정을 수정할 수도 있음



3. 수정해야하는 요소들 (display.h에 defined)
    - lookahead distance
        > 도달해야만 curve fitting 사용되는 waypoint 그룹 업데이트 됨
    - MATRIX_ROW
        > 해당 수 만큼 curve fitting시 사용되는 점 갯수 달라짐
        > MATRIX_ROW+20개 만큼 점 사용됨 (+20은 수정해야될수도 있음)
        > 자세한 사항은 display.cpp에서 SetMatrixCols 함수 참고
    
    - 도착하기 전에 segmentation fault 발생 문제
        > 마지막 waypoint 이후의 index가 계산에 사용되지 않도록 수정 필요함

화이팅