#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

class ObstacleAvoidance {
public:
    ObstacleAvoidance() 
        : forward_speed_(1000.0),  // 장애물이 없을 때의 속도
          stop_speed_(0.0),        // 장애물이 있을 때의 속도
          obstacle_distance_threshold_(1.0) { // 장애물 감지 거리 (단위: m)

        // 퍼블리셔와 서브스크라이버 초기화
        control_speed_pub_ = nh_.advertise<std_msgs::Float64>("/commands/motor/speed", 10);
        lidar_sub_ = nh_.subscribe("/scan", 10, &ObstacleAvoidance::lidarCallback, this);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        bool obstacle_detected = false;

        // 앞쪽 특정 각도 범위에 대한 레이저 데이터 검사
        int min_index = msg->ranges.size() / 3;           // 앞쪽 60도 범위의 시작 인덱스
        int max_index = 2 * msg->ranges.size() / 3;       // 앞쪽 60도 범위의 끝 인덱스

        // 장애물 감지 로직
        for (int i = min_index; i < max_index; i++) {
            if (msg->ranges[i] < obstacle_distance_threshold_) {
                obstacle_detected = true;
                break;
            }
        }

        std_msgs::Float64 cmd_mtr_spd;
        if (obstacle_detected) {
            cmd_mtr_spd.data = stop_speed_;
            ROS_INFO("Obstacle detected! Stopping the robot.");
        } else {
            cmd_mtr_spd.data = forward_speed_;
            ROS_INFO("Path is clear. Moving forward.");
        }

        // 속도 퍼블리시
        control_speed_pub_.publish(cmd_mtr_spd);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher control_speed_pub_;
    ros::Subscriber lidar_sub_;

    double forward_speed_;           // 장애물이 없을 때의 속도
    double stop_speed_;              // 장애물이 있을 때의 속도
    double obstacle_distance_threshold_;  // 장애물 감지 거리
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance_node");

    // ObstacleAvoidance 클래스 인스턴스 생성
    ObstacleAvoidance obstacle_avoidance;

    ros::spin();  // 메시지 처리 루프
    return 0;
}
