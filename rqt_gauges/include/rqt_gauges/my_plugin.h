
#ifndef my_plugin_H
#define my_plugin_H

#include <rqt_gui_cpp/plugin.h>
//#include <ui_my_plugin.h> // What is this?!
#include <QWidget>

namespace rqt_gauges {

class my_plugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  my_plugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::my_plugin ui_;
  QWidget* widget_;
};
}  // namespace rqt_gauges
#endif  // my_plugin_H

/*
#ifndef my_plugin_H
#define my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <Qmy_plugin>
#include "qcgaugewidget.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace Ui {
class my_plugin;
}

class my_plugin : public Qmy_plugin
{
    Q_OBJECT

public:
    explicit my_plugin(QWidget *parent = 0, ros::NodeHandle* nh = 0);
    ~my_plugin();
    QcNeedleItem *mSpeedNeedle;

private:
    Ui::my_plugin *ui;

    QcGaugeWidget * mSpeedGauge;
};

#endif // my_plugin_H
*/
