



#include "random_drive/random_drive.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{

	rclcpp::init(argc, argv);

	auto node = std::make_shared<RandomDrive_NS::RandomDrive>();

	rclcpp::spin(node);

	rclcpp::shutdown();



	return 0;
}
