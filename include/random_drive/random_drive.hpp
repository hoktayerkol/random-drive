#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;

namespace RandomDrive_NS
{
	class RandomDrive : public rclcpp::Node
	{
		private:
			rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
			rclcpp::TimerBase::SharedPtr timer_;
			geometry_msgs::msg::Twist twist_msg_;

			rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
			sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

			void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
			void control_callback(void);
			bool is_there_obstacle();
			void turn_right();
			void go_forward();
			void stop();

			enum State{FORWARD, RIGHT, STOP};
			State current_state_=STOP;
			State last_state_=STOP;

			const float LINEAR_SPEED = 0.3f;
			const float ANGULAR_SPEED = 0.3f;
			
		public:
			RandomDrive();
	};

} // end of RandomDrive_NS

