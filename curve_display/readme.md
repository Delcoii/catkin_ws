성훈이가 언수에게

y = ax3 + bx2 + cx + d 인 curve에 대해
a, b, c, d(코드에선 a3, a2, a1, a0)를 추측하는 코드임.

todo list
1. include/waypoint_save/에 get_lat_lon.cpp파일에서 lane4th.csv 저장 경로 수정해야함

2. armadillo 라이브러리 잘 깔아야함

3. 수정해야하는 요소들 (display.h defined)
    - lookahead distance
        > 도달해야만 curve fitting 사용되는 waypoint 그룹 업데이트 됨
    - MATRIX_ROW
        > 해당 수 만큼 curve fitting시 사용되는 점 갯수 달라짐
        > MATRIX_ROW+20개 만큼 점 사용됨 (+20은 수정해야될수도 있음)
        > 자세한 사항은 display.cpp에서 SetMatrixCols 함수 참고
    
    - 도착하기 전에 segmentation fault 발생 문제
        > 마지막 waypoint 이후의 index가 계산에 사용되지 않도록 수정 필요함

화이팅