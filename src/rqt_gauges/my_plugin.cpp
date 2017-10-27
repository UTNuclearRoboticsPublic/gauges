
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <rqt_gauges/my_plugin.h>
#include <rqt_gauges/qcgaugewidget.h>
#include <stdlib.h>
#include <sstream>

namespace rqt_gauges {

// Callback to change the position of the needle
void MyPlugin::newDataCallback(const std_msgs::Float64& msg)
{
  mSpeedNeedle_-> setCurrentValue(msg.data);
}

// Constructor is called first before initPlugin function, needless to say.
MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // give QObjects reasonable names
  setObjectName("Gauge");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  //QStringList argv = context.argv();

  // create QWidget
  mSpeedGauge_ = new QcGaugeWidget();
  widget_ = mSpeedGauge_; // This works because mSpeedGauge is inherited from a QWidget class, I guess
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // Window title
  widget_->setWindowTitle("Gauge[*]");
  if (context.serialNumber() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }

  // Set default parameters for the gauge. To customize, these can be overwritten.
  // See http://wiki.ros.org/Parameter%20Server
  // The parameters are numbered so multiple gauges can be launched.

  std::string gaugeNum;
  std::stringstream out;
  out << context.serialNumber();
  gaugeNum = out.str();

  // Set each parameter if it isn't defined already
  if (!getNodeHandle().hasParam("topic"+gaugeNum))
    getNodeHandle().setParam("topic"+gaugeNum, "/roll");
  if (!getNodeHandle().hasParam("gauge_name"+gaugeNum))
    getNodeHandle().setParam("gauge_name"+gaugeNum, "Roll");
  if (!getNodeHandle().hasParam("minimum"+gaugeNum))
    getNodeHandle().setParam("minimum"+gaugeNum, 0);
  if (!getNodeHandle().hasParam("maximum"+gaugeNum))
    getNodeHandle().setParam("maximum"+gaugeNum, 100);
  if (!getNodeHandle().hasParam("danger_threshold"+gaugeNum))
    getNodeHandle().setParam("danger_threshold"+gaugeNum, 50);
  if (! getNodeHandle().hasParam("pixel_size"+gaugeNum))
    getNodeHandle().setParam("pixel_size"+gaugeNum, 200); // width and height

  // Set up the gauge
  int gauge_size = 150;
  getNodeHandle().getParam("pixel_size"+gaugeNum, gauge_size);
  mSpeedGauge_->setFixedHeight(gauge_size);
  mSpeedGauge_->setFixedWidth(gauge_size);
  mSpeedGauge_->addBackground(99);
  QcBackgroundItem *bkg1 = mSpeedGauge_->addBackground(92);
  bkg1->clearrColors();
  bkg1->addColor(0.1,Qt::black);
  bkg1->addColor(1.0,Qt::white);

  QcBackgroundItem *bkg2 = mSpeedGauge_->addBackground(88);
  bkg2->clearrColors();
  bkg2->addColor(0.1,Qt::gray);
  bkg2->addColor(1.0,Qt::darkGray);

  // Gauge label
  std::string gaugeName;
  getNodeHandle().getParam("gauge_name"+gaugeNum, gaugeName);
  mSpeedGauge_->addLabel(70)->setText(gaugeName.c_str());

  // Range of the indicator
  int minimum = 0;
  getNodeHandle().getParam("minimum"+gaugeNum, minimum);
  int maximum = 100;
  getNodeHandle().getParam("maximum"+gaugeNum, maximum);
  //mSpeedGauge->addArc(55);
  mSpeedGauge_->addDegrees(65)->setValueRange(minimum, maximum);
  //mSpeedGauge->addColorBand(50);
  mSpeedGauge_->addValues(80)->setValueRange(minimum, maximum);

  // The needle
  QcLabelItem *lab = mSpeedGauge_->addLabel(40);
  lab->setText("0");
  mSpeedNeedle_ = mSpeedGauge_->addNeedle(60);
  mSpeedNeedle_->setLabel(lab);
  mSpeedNeedle_->setColor(Qt::black);
  mSpeedNeedle_->setValueRange(minimum,maximum);
  mSpeedGauge_->addBackground(7);
  mSpeedGauge_->addGlass(88);

  // Color band. Red starts at "danger_threshold"
  double threshold = 0.;
  getNodeHandle().getParam("danger_threshold"+gaugeNum, threshold);
  mSpeedGauge_->addColorBand(threshold);

  // add widget to the user interface
  context.addWidget(widget_);

  // Subscribe to new data
  std::string topicName;
  getNodeHandle().getParam("topic"+gaugeNum, topicName);
  needleSub_ = getNodeHandle().subscribe (topicName, 1, &rqt_gauges::MyPlugin::newDataCallback, this);
}

void MyPlugin::shutdownPlugin()
{
  needleSub_.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_gauges::MyPlugin, rqt_gui_cpp::Plugin)
