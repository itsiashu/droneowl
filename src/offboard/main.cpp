#include "offboard_control.cpp"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));

    rclcpp::shutdown();
    return 0;
}
