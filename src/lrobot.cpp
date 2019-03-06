#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <QTableWidget>
#include <QTabWidget>
#include <QtUiTools/QUiLoader>
#include <QFile>
#include "lrobot.h"
#include "tabwidget.h"
#include <vector>
#include <memory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
//ADD

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometry_msgs/PointStamped.h>

#define  PI  3.14159265358979323846

using namespace std;
namespace rviz_lrobot_commander
{


// 构造函数，初 始化变量
LROBOT::LROBOT( QWidget* parent )
  : rviz::Panel( parent )

{
  TabWidget* t=new TabWidget;

  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget(t);

//  QTabWidget* mytab = new QTabWidget;
//  QLabel *label1 = new QLabel();//创建一个QLabel（QWidget*）
//  QLabel *label2 = new QLabel();
//  mytab->addTab(new TabWidget1(),"robotrun");
//  mytab->addTab(new TabWidget2(),"calligraphy");
//  topic_layout->addWidget(mytab);
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

}

// 重载父类的功能
void LROBOT::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}


// 重载父类的功能，加载配置数据
void LROBOT::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
   // updateTopic();
  }
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_lrobot_commander::LROBOT,rviz::Panel )
// END_TUTORIAL
