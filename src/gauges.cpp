
#include <mainwindow.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gauge_node");
  ros::NodeHandle nh;

  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  ROS_INFO_STREAM("Exiting");
  return a.exec();
}
