#include "ros/ros.h"                           // 패키지의 기본 헤더파일
#include "easy_tutorial/MsgTutorial.h"         // MsgTutorial 메세지 파일 헤더
                                               // (빌드 후 자동 생성됨)
int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_publisher");  // 노드명 초기화
    ros::NodeHandle nh;                        // 노드 핸들 선언

    // 토픽명 easy tutorial
    // 메세지파일 MsgTutorial.msg
    // publisher 큐 size 100
    
    ros::Publisher ros_tutorial_pub = nh.advertise<easy_tutorial::MsgTutorial>("ros_tutorial_msg", 100);
    
    // loop 주기 10Hz. 0.1초 반복
    ros::Rate loop_rate(10);
    
    int count = 0;

    easy_tutorial::MsgTutorial msg;
    
    while(ros::ok())
    {
        msg.stamp = ros::Time::now();
        msg.data = count;

        ROS_INFO("send msg = %d", msg.stamp.sec);
        ROS_INFO("send msg = %d", msg.stamp.nsec);
        ROS_INFO("send msg = %d", msg.data);

        ros_tutorial_pub.publish(msg);

        loop_rate.sleep();

        ++count;
    }
    
    return 0;
}
