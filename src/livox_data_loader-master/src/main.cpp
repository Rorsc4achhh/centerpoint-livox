#include "data_loader.hpp"

int main(int argc, char **argv)
{
   // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
   rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<DataLoader>());

  rclcpp::shutdown();

    return 0;
}

