
#include <mainwindow.h>
#include <QApplication>

namespace gauges
{
  QcNeedleItem *theNeedle; // Get access to this Qt member
}

void newDataCallback(const std_msgs::Float64& msg);

int main(int argc, char** argv) {
  ros::init(argc, argv, "gauge_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner* spinner;
  spinner = new ros::AsyncSpinner(1);
  spinner->start();
  
  // Subscribe to new data
  std::string topicName;
  nh.getParam("topic", topicName);
  ros::Subscriber listener = nh.subscribe(topicName, 1, newDataCallback);
  
  QApplication qtApp(argc, argv);
  MainWindow window(0, &nh);
  window.show();
  gauges::theNeedle = window.mSpeedNeedle; // Get access to this Qt member
  
  // Show the graphic. This is blocking and it must be in the main thread,
  // so I used multithreaded ROS spinners to get around it.
  qtApp.exec();
  
  // Multithreaded spinner
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}

void newDataCallback(const std_msgs::Float64& msg)
{
  gauges::theNeedle -> setCurrentValue(msg.data);
}
