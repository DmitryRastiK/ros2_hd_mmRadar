#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "hd_radar_gui/gui_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hd_radar_gui_node");

  RadarGuiWidget widget(node);
  widget.setWindowTitle("HD Radar GUI");
  widget.resize(400, 300);
  widget.show();

  // Spin ROS in a separate thread so Qt remains responsive
  std::thread ros_thread([](){ rclcpp::spin(rclcpp::Node::make_shared("dummy_spinner")); });
  ros_thread.detach();

  int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}


