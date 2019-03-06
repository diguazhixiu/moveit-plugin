#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <QDialog>


class QLineEdit;
class QFileInfo;
//class QTabWidget;
class QDialogButtonBox;
class QWidget;
namespace rviz_lrobot_commander
{
// 所有的plugin都必须是rviz::Panel的子类
class LROBOT: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  LROBOT( QWidget* parent = 0 );

  // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
  // 中，数据就是topic的名称
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // 公共槽.
public Q_SLOTS:

  // 内部槽.
protected Q_SLOTS:

  // 内部变量.
protected:

  // ROS的publisher，用来发布速度topic
  ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace rviz_lrobot_commander

#endif // TELEOP_PANEL_H
