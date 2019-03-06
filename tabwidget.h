#ifndef TABWIDGET_H
#define TABWIDGET_H

#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <Eigen/Eigen>
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
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PointStamped.h>
#include <QTabWidget>
#include <QButtonGroup>
#include <string>
#include <fstream>


//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgproc/imgproc_c.h"
using namespace std;
using namespace Eigen;
namespace Ui {
class TabWidget;
}

class TabWidget : public QTabWidget
{
  Q_OBJECT

public:
  explicit TabWidget(QWidget *parent = 0);
  ~TabWidget();

  Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);
  Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);
  Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);
  Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);
  Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);
  Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);
  float TimeToZHENG(float t);
  float TimeToZSHU(float t);
  float TimeToZPIE(float t);
  float TimeToZNA(float t);
  float TimeToZDIAN(float t);
  void RoutePlanYong(void);
  float FtoZ(float Fz);
  void RoutePlanDIAN(void);
  void Tibi(float nextx,float nexty);
  void BiHuaHeng(float beginx,float beginy,float nextx,float nexty,bool needzhe);
  void BiHuaShu(float beginx,float beginy,float nextx,float nexty,bool needgou);
  void BiHuaPie(float beginx,float beginy,float nextx,float nexty);
  void BiHuaNa(float beginx,float beginy,float nextx,float nexty);
  void Productoffline();
  std::vector <float> storex;
  std::vector <float> storey;
  std::vector <float> storez;
  float num;
  float px;
  float py;
  float pz;
  float ow;
  float ox;
  float oy;
  float oz;
  double pxgo;
  double pygo;
  double pzgo;
  double owgo;
  double oxgo;
  double oygo;
  double ozgo;
  double joint1go;
  double joint2go;
  double joint3go;
  double joint4go;
  double joint5go;
  double joint6go;
  double toolx;
  double tooly;
  double toolz;
  double toolrx;
  double toolry;
  double toolrz;
  double workx;
  double worky;
  double workz;
  double workrx;
  double workry;
  double workrz;
  double jumpx;
  double jumpy;
  double jumpz;
  double jumprx;
  double jumpry;
  double jumprz;

  std::vector <double> vecjumpxbegin;
  std::vector <double> vecjumpybegin;
  std::vector <double> vecjumpzbegin;
  std::vector <double> vecjumprxbegin;
  std::vector <double> vecjumprybegin;
  std::vector <double> vecjumprzbegin;
  std::vector <double> vecjumpxend;
  std::vector <double> vecjumpyend;
  std::vector <double> vecjumpzend;
  std::vector <double> vecjumprxend;
  std::vector <double> vecjumpryend;
  std::vector <double> vecjumprzend;
  double stepsize;
  double velscale;
  std::vector <double> recordpx;
  std::vector <double> recordpy;
  std::vector <double> recordpz;
  std::vector <double> recordow;
  std::vector <double> recordox;
  std::vector <double> recordoy;
  std::vector <double> recordoz;
  int recordnum;
  int jumpbeginnum;
  int jumpendnum;
  QTimer *timer;
  QButtonGroup *robotmodegroup;
  int robotmode;
  QImage image;
private Q_SLOTS:

  void on_editor_pxgo_editingFinished();

  void on_editor_pygo_editingFinished();

  void on_editor_pzgo_editingFinished();


  void on_editor_j1go_editingFinished();

  void on_editor_j2go_editingFinished();

  void on_editor_j3go_editingFinished();

  void on_editor_j4go_editingFinished();

  void on_editor_j5go_editingFinished();

  void on_editor_j6go_editingFinished();

  void on_lineEdit_toolx_editingFinished();

  void on_lineEdit_tooly_editingFinished();

  void on_lineEdit_toolz_editingFinished();

  void on_lineEdit_toolrx_editingFinished();

  void on_lineEdit_toolry_editingFinished();

  void on_lineEdit_toolrz_editingFinished();

  void on_lineEdit_workx_editingFinished();

  void on_lineEdit_worky_editingFinished();

  void on_lineEdit_workz_editingFinished();

  void on_lineEdit_workrx_editingFinished();

  void on_lineEdit_workry_editingFinished();

  void on_lineEdit_workrz_editingFinished();

  void on_lineEdit_jumpx_editingFinished();

  void on_lineEdit_jumpy_editingFinished();

  void on_lineEdit_jumpz_editingFinished();

  void on_lineEdit_jumprx_editingFinished();

  void on_lineEdit_jumpry_editingFinished();

  void on_lineEdit_jumprz_editingFinished();

  void on_pushButton_jointgo_clicked();

  void on_pushButton_gohome_clicked();

  void on_pushButton_posgo_clicked(bool checked);

  void on_pushButton_charactergo_clicked();

  void on_pushButton_poszadd_clicked();

  void on_pushButton_posxadd_clicked();

  void on_pushButton_posxdec_clicked();

  void on_pushButton_posyadd_clicked();

  void on_pushButton_posydec_clicked();

  void on_pushButton_poszdec_clicked();


  void on_lineEdit_stepsize_editingFinished();

  void on_pushButton_posalways_clicked();

  void on_pushButton_posshutdown_clicked();

  void getpostimer();
  void on_pushButton_j1add_clicked();

  void on_pushButton_j1dec_clicked();

  void on_pushButton_j2add_clicked();

  void on_pushButton_j2dec_clicked();

  void on_pushButton_j3add_clicked();

  void on_pushButton_j3dec_clicked();

  void on_pushButton_j4add_clicked();

  void on_pushButton_j4dec_clicked();

  void on_pushButton_j5add_clicked();

  void on_pushButton_j5dec_clicked();

  void on_pushButton_j6add_clicked();

  void on_pushButton_j6dec_clicked();

  void on_pushButton_record_clicked();

  void on_pushButton_recordclear_clicked();

  void on_editor_owgo_editingFinished();

  void on_editor_oxgo_editingFinished();

  void on_editor_oygo_editingFinished();

  void on_editor_ozgo_editingFinished();

  void on_editor_velscale_editingFinished();

  void on_pushButton_simulation_clicked();

  void on_pushButton_jumpbeginadd_clicked();

  void on_pushButton_jumpbegindec_clicked();

  void on_pushButton_jumpendadd_clicked();

  void on_pushButton_jumpenddec_clicked();

  void onRadioClickRobotMode();
  void on_pushButton_Preview_clicked();

private:
  Ui::TabWidget *ui;
};

#endif // TABWIDGET_H
