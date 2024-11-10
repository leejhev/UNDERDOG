#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher control_speed;  // 전역 변수로 Publisher 선언

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    double speed_ = 2000;  // 속도 값
    std_msgs::Float64 cmd_mtr_spd;
    cmd_mtr_spd.data = speed_;
    control_speed.publish(cmd_mtr_spd);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node_ctrl");
    ros::NodeHandle nh;

    // Publisher 초기화
    control_speed = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10, true);

    // Subscriber 초기화
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, lidar_callback);

    ros::spin();  // 메시지 처리 루프
    return 0;
}
