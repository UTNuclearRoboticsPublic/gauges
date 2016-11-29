
#include "my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_gauges {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
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

/*
#include "MyPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <QMessageBox>
#include <QtGui>
#include <QtCore>
#include <QString>
#include <QStringList>

QT_BEGIN_NAMESPACE

class Ui_MyPlugin
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMyPlugin *MyPlugin)
    {
        if (MyPlugin->objectName().isEmpty())
            MyPlugin->setObjectName(QString("MyPlugin"));
        MyPlugin->resize(400, 300);
        centralWidget = new QWidget(MyPlugin);
        centralWidget->setObjectName(QString("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString("horizontalLayout"));

        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);

        MyPlugin->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MyPlugin);
        menuBar->setObjectName(QString("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 20));
        MyPlugin->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MyPlugin);
        mainToolBar->setObjectName(QString("mainToolBar"));
        MyPlugin->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MyPlugin);
        statusBar->setObjectName(QString("statusBar"));
        MyPlugin->setStatusBar(statusBar);

        retranslateUi(MyPlugin);

        QMetaObject::connectSlotsByName(MyPlugin);
    } // setupUi

    void retranslateUi(QMyPlugin *MyPlugin)
    {
        MyPlugin->setWindowTitle(QApplication::translate("MyPlugin", "MyPlugin", 0));
    } // retranslateUi

};

namespace Ui {
    class MyPlugin: public Ui_MyPlugin {};
} // namespace Ui

QT_END_NAMESPACE


MyPlugin::MyPlugin(QWidget *parent, ros::NodeHandle* nh) :
    QMyPlugin(parent),
    ui(new Ui::MyPlugin)
{ 
    ui->setupUi(this);

    mSpeedGauge = new QcGaugeWidget;
    mSpeedGauge->addBackground(99);
    QcBackgroundItem *bkg1 = mSpeedGauge->addBackground(92);
    bkg1->clearrColors();
    bkg1->addColor(0.1,Qt::black);
    bkg1->addColor(1.0,Qt::white);

    QcBackgroundItem *bkg2 = mSpeedGauge->addBackground(88);
    bkg2->clearrColors();
    bkg2->addColor(0.1,Qt::gray);
    bkg2->addColor(1.0,Qt::darkGray);

    // Range of the indicator
    int minimum;
    nh->getParam("minimum", minimum);
    int maximum;
    nh->getParam("maximum", maximum);
    //mSpeedGauge->addArc(55);
    mSpeedGauge->addDegrees(65)->setValueRange(minimum,maximum);
    //mSpeedGauge->addColorBand(50);
    mSpeedGauge->addValues(80)->setValueRange(minimum,maximum);
    
    // Gauge label
    std::string gaugeName;
    nh->getParam("gauge_name", gaugeName);
    mSpeedGauge->addLabel(70)->setText(gaugeName.c_str());
    
    // The needle
    QcLabelItem *lab = mSpeedGauge->addLabel(40);
    lab->setText("0");
    mSpeedNeedle = mSpeedGauge->addNeedle(60);
    mSpeedNeedle->setLabel(lab);
    mSpeedNeedle->setColor(Qt::black);
    mSpeedNeedle->setValueRange(minimum,maximum);
    mSpeedGauge->addBackground(7);
    mSpeedGauge->addGlass(88);
    ui->horizontalLayout->addWidget(mSpeedGauge);
}

MyPlugin::~MyPlugin()
{
    delete ui;
}
*/
