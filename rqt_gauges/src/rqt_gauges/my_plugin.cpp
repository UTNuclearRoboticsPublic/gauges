
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <rqt_gauges/my_plugin.h>
#include <rqt_gauges/qcgaugewidget.h>

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
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  mSpeedGauge_ = new QcGaugeWidget();
  widget_ = mSpeedGauge_; // This works because mSpeedGauge is inherited from a QWidget class, I guess
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);

  // Set up the gauge
  const int gaugeSize = 150;
  mSpeedGauge_->setFixedHeight(gaugeSize);
  mSpeedGauge_->setFixedWidth(gaugeSize);
  mSpeedGauge_->addBackground(99);
  QcBackgroundItem *bkg1 = mSpeedGauge_->addBackground(92);
  bkg1->clearrColors();
  bkg1->addColor(0.1,Qt::black);
  bkg1->addColor(1.0,Qt::white);

  QcBackgroundItem *bkg2 = mSpeedGauge_->addBackground(88);
  bkg2->clearrColors();
  bkg2->addColor(0.1,Qt::gray);
  bkg2->addColor(1.0,Qt::darkGray);

  // Range of the indicator
  int minimum = 0;
  //nh->getParam("minimum", minimum);
  int maximum = 100;
  //nh->getParam("maximum", maximum);
  //mSpeedGauge->addArc(55);
  mSpeedGauge_->addDegrees(65)->setValueRange(minimum,maximum);
  //mSpeedGauge->addColorBand(50);
  mSpeedGauge_->addValues(80)->setValueRange(minimum,maximum);

  // Gauge label
  std::string gaugeName = "Roll";
  //nh->getParam("gauge_name", gaugeName);
  mSpeedGauge_->addLabel(70)->setText(gaugeName.c_str());

  // The needle
  QcLabelItem *lab = mSpeedGauge_->addLabel(40);
  lab->setText("0");
  mSpeedNeedle_ = mSpeedGauge_->addNeedle(60);
  mSpeedNeedle_->setLabel(lab);
  mSpeedNeedle_->setColor(Qt::black);
  mSpeedNeedle_->setValueRange(minimum,maximum);
  mSpeedGauge_->addBackground(7);
  mSpeedGauge_->addGlass(88);

  // add widget to the user interface
  context.addWidget(widget_);

  //ros::NodeHandle nh;
  //rqt_gauges::nhPtr = &nh;
  needleSub_ = getNodeHandle().subscribe ("roll", 1, &rqt_gauges::MyPlugin::newDataCallback, this);
  //rqt_gauges::needleSubPtr = &rqt_gauges::needleSub;
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
PLUGINLIB_DECLARE_CLASS(rqt_gauges, MyPlugin, rqt_gauges::MyPlugin, rqt_gui_cpp::Plugin)
