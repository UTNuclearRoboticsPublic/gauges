
#include <ros/ros.h>
#include <mainwindow.h>
#include <QApplication>

// Standard C++ entry point
int main(int argc, char** argv) {

  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  ros::init(argc, argv, "gauges");
  ros::start();

  ROS_INFO_STREAM("Hello, world!");

  ros::spin();

  ros::shutdown();
  return 0;
}
