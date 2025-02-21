#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SlamTFBroadcaster : public rclcpp::Node {
public:
    SlamTFBroadcaster()
        : Node("slam_tf_broadcaster"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {

        // ✅ Launch 파일에서 IMU 토픽을 동적으로 설정 가능하도록 파라미터 선언
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<double>("base_x", 0.0);
        this->declare_parameter<double>("base_y", 0.0);
        this->declare_parameter<double>("base_z", 0.3);

        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        base_x_ = this->get_parameter("base_x").as_double();
        base_y_ = this->get_parameter("base_y").as_double();
        base_z_ = this->get_parameter("base_z").as_double();

        // ✅ 동적으로 설정된 IMU 토픽을 구독
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&SlamTFBroadcaster::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU TF Broadcaster Initialized. Listening to %s...", imu_topic.c_str());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double base_x_, base_y_, base_z_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // ✅ IMU 데이터에서 Quaternion 추출
        tf2::Quaternion imu_quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);  // ✅ Roll, Pitch, Yaw 값 변환

        // 🔹 base_footprint → base_link 변환 설정
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base_footprint";  // ✅ 기준 프레임
        transformStamped.child_frame_id = "base_link";  // ✅ 변환 적용할 프레임

        // 🔹 변환 값 설정 (IMU는 위치 정보가 없으므로 translation은 0으로 설정)
        transformStamped.transform.translation.x = base_x_;
        transformStamped.transform.translation.y = base_y_;
        transformStamped.transform.translation.z = base_z_;  

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);  // ✅ IMU 데이터를 기반으로 변환 적용
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // 🔹 TF 브로드캐스트
        tf_broadcaster_->sendTransform(transformStamped);

        // ✅ 로그 출력 (디버깅)
        RCLCPP_INFO(this->get_logger(), "Updated TF: base_footprint -> base_link | X: %.3f, Y: %.3f, Z: %.3f | Roll: %.3f, Pitch: %.3f, Yaw: %.3f",
                    base_x_, base_y_, base_z_, roll, pitch, yaw);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
