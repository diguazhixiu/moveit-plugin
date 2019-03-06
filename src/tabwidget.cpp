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

//#include "lrobot.h"

#include "tabwidget.h"
#include "ui_tabwidget.h"
#include <QMessageBox>
#include "workthreadhome.h"

#define  PI  3.14159265358979323846
#define  GOHOME  0
#define  J1ADD  1
#define  J1DEC  2
#define  J2ADD  3
#define  J2DEC  4
#define  J3ADD  5
#define  J3DEC  6
#define  J4ADD  7
#define  J4DEC  8
#define  J5ADD  9
#define  J5DEC  10
#define  J6ADD  11
#define  J6DEC  12
#define  JOINTGO  13
#define  POSGO  14
#define  POSXADD  15
#define  POSXDEC  16
#define  POSYADD  17
#define  POSYDEC  18
#define  POSZADD  19
#define  POSZDEC  20
#define  RECORDSIMULATION  21
#define  CHARACTERGO  22
using namespace std;
WorkThreadhome *mythread[2];
TabWidget::TabWidget(QWidget *parent) :
  QTabWidget(parent),
  ui(new Ui::TabWidget),
  recordnum(0),
  jumpbeginnum(0),
  jumpendnum(0),
  robotmode(0)

{
      ui->setupUi(this);
      timer = new QTimer(this);
      mythread[0] = new WorkThreadhome;
      mythread[1] = new WorkThreadhome;
      ui->editor_velscale->setText("1");
      velscale = 1;
      ui->lineEdit_stepsize->setText("10");
      stepsize = 10;
      ui->lineEdit_toolx->setText("-86.768");
      toolx=-86.768;
      ui->lineEdit_tooly->setText("0.739");
      tooly=0.739;
      ui->lineEdit_toolz->setText("99.932");
      toolz=99.932;
      ui->lineEdit_toolrx->setText("0.000");
      toolrx=0.000;
      ui->lineEdit_toolry->setText("-90.000");
      toolry=-90.000;
      ui->lineEdit_toolrz->setText("0.000");
      toolrz=0.000;
      ui->lineEdit_workx->setText("600.000");
      workx=600.000;
      ui->lineEdit_worky->setText("61.570");
      worky=61.570;
      ui->lineEdit_workz->setText("723.590");
      workz=723.590;
      ui->lineEdit_workrx->setText("0.000");
      workrx=0.000;
      ui->lineEdit_workry->setText("0.000");
      workry=0.000;
      ui->lineEdit_workrz->setText("0.000");
      workrz=0.000;
      //ui->tableWidget_point->setRowCount(20);
      ui->tableWidget_point->setColumnCount(8);
      ui->tableWidget_point->setWindowTitle("QTableWidget & Item");
     // ui->tableWidget_point->resize(400, 300);  //设置表格
      ui->tableWidget_point->setSelectionBehavior(QAbstractItemView::SelectRows);
      QStringList header_point;
      header_point<<"Name"<<"PX"<<"PY"<<"PZ"<<"OW"<<"OX"<<"OY"<<"OZ";   //表头
      ui->tableWidget_point->setHorizontalHeaderLabels(header_point);
      
      ui->tableWidget_beginjump->setRowCount(10);
      ui->tableWidget_beginjump->setColumnCount(6);
      ui->tableWidget_beginjump->setWindowTitle("QTableWidget & Item");
     // ui->tableWidget_beginjump->resize(400, 300);  //设置表格
      QStringList header_begin;
      header_begin<<"X"<<"Y"<<"Z"<<"RX"<<"RY"<<"RZ";   //表头
      ui->tableWidget_beginjump->setHorizontalHeaderLabels(header_begin);
      ui->tableWidget_endjump->setRowCount(10);
      ui->tableWidget_endjump->setColumnCount(6);
      ui->tableWidget_endjump->setWindowTitle("QTableWidget & Item");
      //ui->tableWidget_endjump->resize(400, 300);  //设置表格
      QStringList header_end;
      header_end<<"Name"<<"Age";   //表头
      ui->tableWidget_endjump->setHorizontalHeaderLabels(header_end);
      robotmodegroup = new QButtonGroup(this);
      robotmodegroup -> addButton(ui->radioButton_fangzhen,0);
      robotmodegroup -> addButton(ui->radioButton_lixian,1);
      robotmodegroup -> addButton(ui->radioButton_zaixian,2);
      ui->radioButton_fangzhen->setChecked(true);

      robotmodegroup ->setExclusive(true);

      image = QImage(430,420,QImage::Format_RGB32);  //画布的初始化大小设为800*500，使用32位颜色
          QColor backColor = qRgb(240,255,255);    //画布初始化背景色使用白色
          image.fill(backColor);

      connect(timer, SIGNAL(timeout()), this, SLOT(getpostimer()));
      connect(ui->radioButton_fangzhen,SIGNAL(clicked()), this, SLOT(onRadioClickRobotMode()));
      connect(ui->radioButton_lixian,SIGNAL(clicked()), this, SLOT(onRadioClickRobotMode()));
      connect(ui->radioButton_zaixian,SIGNAL(clicked()), this, SLOT(onRadioClickRobotMode()));
      //group("arm");
}

TabWidget::~TabWidget()
{
  delete ui;
}
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;

    return q;
}
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;

}
Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    return R;
}
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    return q;
}
Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d R = q.matrix();

    return R;
}
Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
    cout << "RotationMatrix2euler result is:" << endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
    return euler;
}



void TabWidget::on_editor_j1go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j1go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint1go = tempfloat;
    mythread[1]->joint1go = joint1go;
}

void TabWidget::on_editor_j2go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j2go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint2go = tempfloat;
    mythread[1]->joint2go = joint2go;
}

void TabWidget::on_editor_j3go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j3go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint3go = tempfloat;
    mythread[1]->joint3go = joint3go;
}

void TabWidget::on_editor_j4go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j4go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint4go = tempfloat;
    mythread[1]->joint4go = joint4go;
}

void TabWidget::on_editor_j5go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j5go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint5go = tempfloat;
    mythread[1]->joint5go = joint5go;
}

void TabWidget::on_editor_j6go_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_j6go->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    joint6go = tempfloat;
    mythread[1]->joint6go = joint6go;
}

void TabWidget::on_lineEdit_toolx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_toolx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    toolx = tempfloat;
}

void TabWidget::on_lineEdit_tooly_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_tooly->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    tooly = tempfloat;
}

void TabWidget::on_lineEdit_toolz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_toolz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    toolz = tempfloat;
}

void TabWidget::on_lineEdit_toolrx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_toolrx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    toolrx = tempfloat;
}

void TabWidget::on_lineEdit_toolry_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_toolry->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    toolry = tempfloat;
}

void TabWidget::on_lineEdit_toolrz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_toolrz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    toolrz = tempfloat;
}

void TabWidget::on_lineEdit_workx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_workx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    workx = tempfloat;
}

void TabWidget::on_lineEdit_worky_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_worky->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    worky = tempfloat;
}

void TabWidget::on_lineEdit_workz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_workz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    workz = tempfloat;
}

void TabWidget::on_lineEdit_workrx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_workrx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    workrx = tempfloat;
}

void TabWidget::on_lineEdit_workry_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_workry->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    workry = tempfloat;
}

void TabWidget::on_lineEdit_workrz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_workrz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    workrz = tempfloat;
}

void TabWidget::on_lineEdit_jumpx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumpx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumpx = tempfloat;
}

void TabWidget::on_lineEdit_jumpy_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumpy->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumpy = tempfloat;
}

void TabWidget::on_lineEdit_jumpz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumpz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumpz = tempfloat;
}

void TabWidget::on_lineEdit_jumprx_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumprx->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumprx = tempfloat;
}

void TabWidget::on_lineEdit_jumpry_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumpry->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumpry = tempfloat;
}

void TabWidget::on_lineEdit_jumprz_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_jumprz->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    jumprz = tempfloat;
}



void TabWidget::on_pushButton_jointgo_clicked()
{
  mythread[1]->threadmode = JOINTGO;
  mythread[1]->start();
}

void TabWidget::on_pushButton_gohome_clicked()
{
  mythread[1]->threadmode = GOHOME;
//  mythread[1]->velscale = velscale;
  mythread[1]->start();
}

void TabWidget::on_pushButton_posgo_clicked(bool checked)
{
  mythread[1]->threadmode = POSGO;
  mythread[1]->start();
}
float TabWidget::TimeToZHENG(float t)
{
  float Fz;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float z;
  if (t<=30)
  {
    p1 =   2.532e-09  ;
    p2 =  -3.056e-07  ;
    p3 =   1.481e-05  ;
    p4 =  -0.0003678  ;
    p5 =    0.004959  ;
    p6 =    -0.03568  ;
    p7 =      0.1301  ;
    p8 =     -0.2589  ;
    p9 =     0.02146  ;
    Fz= -(p1*t*t*t*t*t*t*t*t + p2*t*t*t*t*t*t*t + p3*t*t*t*t*t*t + p4*t*t*t*t*t + p5*t*t*t*t + p6*t*t*t + p7*t*t + p8*t + p9);
  }
  if (t>30&&t<=70)
  {
    Fz=0.3;
  }
  if (t>70)
  {
    p1 =    3.77e-07;
    p2 =  -2.197e-05;
    p3 =   0.0001758;
    p4 =     0.00855;
    p5 =     -0.1197;
    p6 =     -0.2409;
    Fz=-(p1*(t-70)*(t-70)*(t-70)*(t-70)*(t-70) + p2*(t-70)*(t-70)*(t-70)*(t-70) + p3*(t-70)*(t-70)*(t-70) + p4*(t-70)*(t-70) + p5*(t-70) + p6);


  }
  z=FtoZ(Fz);
  return z;
}
float TabWidget::TimeToZSHU(float t)
{
  float Fz;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float z;
  if (t<=20)
  {
    p1 =   -1.56e-07;
    p2 =   1.716e-05;
    p3 =  -0.0006172;
    p4 =    0.006483;
    p5 =     0.03406;
    p6 =    -0.01635;

    Fz= (p1*t*t*t*t*t + p2*t*t*t*t + p3*t*t*t + p4*t*t + p5*t + p6);
  }
  if (t>20&&t<=70)
  {
    Fz=0.35;
  }
  if (t>70)
  {
    p1 =  -7.499e-07;
    p2 =   5.751e-05;
    p3 =   -0.001433;
    p4 =       0.011;
    p5 =     0.01095;
    p6 =      0.1872;

    Fz=(p1*(t-70)*(t-70)*(t-70)*(t-70)*(t-70) + p2*(t-70)*(t-70)*(t-70)*(t-70) + p3*(t-70)*(t-70)*(t-70) + p4*(t-70)*(t-70) + p5*(t-70) + p6);

  }
  z=FtoZ(Fz);
  return z;
}
float TabWidget::TimeToZPIE(float t)
{
  float Fz;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float z;
  if (t<=2)
  {
    Fz=0.05;
  }
  if (t>2&&t<=70)
  {
    p1 =   2.848e-14;
    p2 =  -1.293e-11;
    p3 =   2.404e-09;
    p4 =  -2.338e-07;
    p5 =   1.252e-05;
    p6 =  -0.0003429;
    p7 =    0.003079;
    p8 =     0.03319;
    p9 =     0.08518;

    Fz= p1*t*t*t*t*t*t*t*t + p2*t*t*t*t*t*t*t + p3*t*t*t*t*t*t + p4*t*t*t*t*t + p5*t*t*t*t + p6*t*t*t + p7*t*t + p8*t + p9;

  }
  if (t>70)
  {
    Fz=0.66666 - t*0.0066666;
  }
  z=FtoZ(Fz);
  return z;
}
float TabWidget::TimeToZNA(float t)
{
  float Fz;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float z;
  if (t<=10)
  {
    Fz=0.2;
  }
  if (t>10)
  {
    p1 =   3.209e-08;
    p2 =  -8.137e-06;
    p3 =    0.000587;
    p4 =   -0.009807;
    p5 =      0.1106;


    Fz= (p1*t*t*t*t + p2*t*t*t + p3*t*t + p4*t + p5);
    if (t<90)
    {
      Fz=Fz+0.35;
    }
  }
// 	if (t>99)
// 	{
// 		Fz=0.1;
// 	}
  z=FtoZ(Fz);
  return z;
}
float TabWidget::TimeToZDIAN(float t)
{
  float Fz;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float p6;
  float p7;
  float p8;
  float p9;
  float z;

  p1 =   5.037e-09;
  p2 =  -1.393e-06;
  p3 =   0.0001361;
  p4 =   -0.005627;
  p5 =     0.09165;
  p6 =    -0.04658;


  Fz= (p1*t*t*t*t*t + p2*t*t*t*t + p3*t*t*t + p4*t*t + p5*t + p6);
  if (t>50)
  {
    Fz=Fz+0.1;
  }
  z=FtoZ(Fz);
  return z;
}

float TabWidget::FtoZ(float Fz)
{
  float z;
  z= 59.44*Fz*Fz*Fz + -64.69*Fz*Fz +35.26*Fz + 0.03238;

  return z;
}
void TabWidget::RoutePlanDIAN(void)
{

  vector <float> startx;
  vector <float> starty;
  vector <float> startz;
  vector <float> goalx;
  vector <float> goaly;
  vector <float> goalz;
  startx.push_back(0);
  goalx.push_back(8);
  starty.push_back(0);
  goaly.push_back(10);

  startx.push_back(8);
  goalx.push_back(8);
  starty.push_back(10);
  goaly.push_back(9);

  startx.push_back(8);
  goalx.push_back(2);
  starty.push_back(9);
  goaly.push_back(2);

  storex.push_back(0);
  storey.push_back(0);
  storez.push_back(0);
  for (int part=0;part<startx.size();part++)
  {
    float chabunum=0;
    if (part==0)
    {
      chabunum = 20;
    }
    if (part==1)
    {
      chabunum = 20;
    }
    if (part==2)
    {
      chabunum = 10;
    }
    for (int i=0;i<chabunum;i++)
    {
      float onepiecex;float onepiecey;
      onepiecex=(goalx[part]-startx[part])/chabunum;
      onepiecey=(goaly[part]-starty[part])/chabunum;


      storex.push_back(storex[storex.size()-1]+onepiecex);
      storey.push_back(storey[storey.size()-1]+onepiecey);

    }

  }
  for (int i=0;i<50;i++)
  {

    num = i/0.5;
    storez.push_back(TimeToZDIAN(num));
  }
}


void TabWidget::Tibi(float nextx,float nexty)
{
  //从上一笔画结束时，提笔
  storex.push_back(storex[storex.size()-1]);
  storey.push_back(storey[storey.size()-1]);
  storez.push_back(-15);
  //移动到下一笔画开始
  storex.push_back(nextx);
  storey.push_back(nexty);
  storez.push_back(-15);

}
void TabWidget::BiHuaHeng(float beginx,float beginy,float nextx,float nexty,bool needzhe)
{
  //nexty就是长度


  vector <float> startx;
  vector <float> starty;
  vector <float> startz;
  vector <float> goalx;
  vector <float> goaly;
  vector <float> goalz;
  startx.push_back(beginx);
  starty.push_back(beginy);
  goalx.push_back(4+beginx);
  goaly.push_back(4+beginy);

  startx.push_back(4+beginx);
  starty.push_back(4+beginy);
  goalx.push_back(nextx+beginx);//0
  goaly.push_back(nexty+beginy);//80

  startx.push_back(nextx+beginx);//0
  starty.push_back(nexty+beginy);//80
  goalx.push_back(2+nextx+beginx);//7
  goaly.push_back(2+nexty+beginy);//85

  startx.push_back(2+nextx+beginx);//7
  starty.push_back(2+nexty+beginy);//85
  goalx.push_back(nextx+beginx);//0
  goaly.push_back(-2+nexty+beginy);//75

  storex.push_back(beginx);
  storey.push_back(beginy);
  storez.push_back(0);
  for (int part=0;part<startx.size();part++)
  {
    float chabunum=0;
    if (part==0)
    {
      chabunum = 40;
    }
    if (part==1)
    {
      chabunum = 100;
    }
    if (part==2)
    {
      chabunum = 20;
    }
    if (part==3)
    {
      chabunum = 30;
    }
    for (int i=0;i<chabunum;i++)
    {

      float onepiecex;float onepiecey;
      onepiecex=(goalx[part]-startx[part])/chabunum;
      onepiecey=(goaly[part]-starty[part])/chabunum;

      if (needzhe==true)
      {
        if (part<2)
        {
          storex.push_back(storex[storex.size()-1]+onepiecex);
          storey.push_back(storey[storey.size()-1]+onepiecey);
        }
      }
      else
      {
        storex.push_back(storex[storex.size()-1]+onepiecex);
        storey.push_back(storey[storey.size()-1]+onepiecey);
      }

    }

  }
  for (int i=0;i<190;i++)
  {

    num = i/1.9;
    if (needzhe==true)
    {
      if (i<140)
      {
        storez.push_back(TimeToZHENG(num));
      }
    }
    else
    {
      storez.push_back(TimeToZHENG(num));
    }

  }
}
void TabWidget::BiHuaShu(float beginx,float beginy,float nextx,float nexty,bool needgou)
{
  //nextx长度


  vector <float> startx;
  vector <float> starty;
  vector <float> startz;
  vector <float> goalx;
  vector <float> goaly;
  vector <float> goalz;
  startx.push_back(0+beginx);
  goalx.push_back(5+beginx);
  starty.push_back(0+beginy);
  goaly.push_back(6+beginy);

  startx.push_back(5+beginx);
  goalx.push_back(5+beginx);
  starty.push_back(6+beginy);
  goaly.push_back(4+beginy);

  startx.push_back(5+beginx);//5
  goalx.push_back(nextx+beginx);//100
  starty.push_back(4+beginy);//4
  goaly.push_back(nexty+beginy);//4

  startx.push_back(nextx+beginx);//100
  goalx.push_back(4+nextx+beginx);//104
  starty.push_back(nexty+beginy);//4
  goaly.push_back(2+nexty+beginy);//6

  startx.push_back(4+nextx+beginx);//104
  goalx.push_back(-4+nextx+beginx);//96
  starty.push_back(2+nexty+beginy);//6
  goaly.push_back(nexty+beginy);//4

  storex.push_back(beginx);
  storey.push_back(beginy);
  storez.push_back(0);
  for (int part=0;part<startx.size();part++)
  {
    float chabunum=0;
    if (part==0)
    {
      chabunum = 40;
    }
    if (part==1)
    {
      chabunum = 40;
    }
    if (part==2)
    {
      chabunum = 80;
    }
    if (part==3)
    {
      chabunum = 30;
    }
    if (part==4)
    {
      chabunum = 40;
    }
    for (int i=0;i<chabunum;i++)
    {
      float onepiecex;float onepiecey;
      onepiecex=(goalx[part]-startx[part])/chabunum;
      onepiecey=(goaly[part]-starty[part])/chabunum;


      storex.push_back(storex[storex.size()-1]+onepiecex);
      storey.push_back(storey[storey.size()-1]+onepiecey);

    }

  }
  for (int i=0;i<230;i++)
  {

    num = i/2.3;
    storez.push_back(TimeToZSHU(num));

  }
  if (needgou==true)
  {
    storex.push_back(nextx+beginx);
    storey.push_back(nexty+beginy);
    storez.push_back(8);
    storex.push_back(-10+nextx+beginx);
    storey.push_back(-10+nexty+beginy);
    storez.push_back(5);
    storex.push_back(-12+nextx+beginx);
    storey.push_back(-12+nexty+beginy);
    storez.push_back(0);
  }
}
void TabWidget::BiHuaPie(float beginx,float beginy,float nextx,float nexty)
{


  vector <float> startx;
  vector <float> starty;
  vector <float> startz;
  vector <float> goalx;
  vector <float> goaly;
  vector <float> goalz;
  startx.push_back(0+beginx);
  goalx.push_back(2+beginx);
  starty.push_back(0+beginy);
  goaly.push_back(2+beginy);

  startx.push_back(2+beginx);
  goalx.push_back(nextx+beginx);//60
  starty.push_back(2+beginy);
  goaly.push_back(nexty+beginy);//-45


  storex.push_back(beginx);
  storey.push_back(beginy);
  storez.push_back(0);
  for (int part=0;part<startx.size();part++)
  {
    float chabunum=0;
    if (part==0)
    {
      chabunum = 20;
    }
    if (part==1)
    {
      chabunum = 80;
    }

    for (int i=0;i<chabunum;i++)
    {
      float onepiecex;float onepiecey;
      onepiecex=(goalx[part]-startx[part])/chabunum;
      onepiecey=(goaly[part]-starty[part])/chabunum;


      storex.push_back(storex[storex.size()-1]+onepiecex);
      storey.push_back(storey[storey.size()-1]+onepiecey);

    }

  }
  for (int i=0;i<100;i++)
  {

    num = i/1;
    storez.push_back(TimeToZPIE(num));
  }
}
void TabWidget::BiHuaNa(float beginx,float beginy,float nextx,float nexty)
{


  vector <float> startx;
  vector <float> starty;
  vector <float> startz;
  vector <float> goalx;
  vector <float> goaly;
  vector <float> goalz;
  startx.push_back(0+beginx);
  goalx.push_back(nextx+beginx);//45
  starty.push_back(0+beginy);
  goaly.push_back(nexty+beginy);//43

  startx.push_back(nextx+beginx);//45
  goalx.push_back(-2+nextx+beginx);//43
  starty.push_back(nexty+beginy);//43
  goaly.push_back(9+nexty+beginy);//52


  storex.push_back(beginx);
  storey.push_back(beginy);
  storez.push_back(0);
  for (int part=0;part<startx.size();part++)
  {
    float chabunum=0;
    if (part==0)
    {
      chabunum = 60;
    }
    if (part==1)
    {
      chabunum = 40;
    }

    for (int i=0;i<chabunum;i++)
    {
      float onepiecex;float onepiecey;
      onepiecex=(goalx[part]-startx[part])/chabunum;
      onepiecey=(goaly[part]-starty[part])/chabunum;


      storex.push_back(storex[storex.size()-1]+onepiecex);
      storey.push_back(storey[storey.size()-1]+onepiecey);

    }

  }
  for (int i=0;i<100 ;i++)
  {

    num = i/1;
    storez.push_back(TimeToZNA(num));
  }
}
void TabWidget::RoutePlanYong(void)
{
  RoutePlanDIAN();
  Tibi(21,-12);
  BiHuaHeng(21,-12,0,18,true);
  BiHuaShu(19,5,67,4,true);
  Tibi(40,-18);
  BiHuaHeng(40,-18,0,13,true);
  BiHuaPie(40,-5,38,-20);
  Tibi(30,24);
  BiHuaPie(30,24,14,-16);
  Tibi(44,8);
  BiHuaNa(44,8,30,34);
}

void TabWidget::on_pushButton_charactergo_clicked()
{
  switch(robotmode)
     {
     case 0:
         qDebug() << QString::fromLocal8Bit("仿真模式");
         mythread[1]->threadmode = CHARACTERGO;
         storex.clear();
         storey.clear();
         storez.clear();
         mythread[1]->storex.clear();
         mythread[1]->storey.clear();
         mythread[1]->storez.clear();
         RoutePlanYong();
         mythread[1]->storex=storex;
         mythread[1]->storey=storey;
         mythread[1]->storez=storez;
         mythread[1]->start();
         break;
     case 1:
         qDebug() << QString::fromLocal8Bit("离线模式");
         storex.clear();
         storey.clear();
         storez.clear();
         RoutePlanYong();
         Productoffline();
         break;
     case 2:
         qDebug() << QString::fromLocal8Bit("在线模式");
         robotmode = 2;
         break;
     }

}

void TabWidget::Productoffline()
{
  ofstream file;
  file.open("/home/rosfun/cx.txt",ios::trunc);
  file<<"PRODUCTID=L"<<endl;
//  file<<"TOOL=-86.768,0.739,99.932,0.000,-90,0"<<endl;
  file<<"TOOL="<<fixed<<setprecision(3)<<toolx<<","<<fixed<<setprecision(3)<<tooly<<","<<fixed<<setprecision(3)<<toolz<<","<<fixed<<setprecision(3)<<toolrx<<","<<fixed<<setprecision(3)<<toolry<<","<<fixed<<setprecision(3)<<toolrz<<","<<endl;
  file<<"HOME=0,0,90,0,45,0"<<endl;
  file<<"OFFLINE=TRUE"<<endl;
  file<<"TOTALELEMENTS=1"<<endl;
//  file<<"FRAME=600,61.57,723.59,0,0,0"<<endl;
  file<<"FRAME="<<fixed<<setprecision(3)<<workx<<","<<fixed<<setprecision(3)<<worky<<","<<fixed<<setprecision(3)<<workz<<","<<fixed<<setprecision(3)<<workrx<<","<<fixed<<setprecision(3)<<workry<<","<<fixed<<setprecision(3)<<workrz<<","<<endl;
  file<<"OFFSET=0,0,0,0,0,0"<<endl;
  file<<"PRESET=StaubliPreset"<<endl;
  file<<"BLEND=1"<<endl;
  file<<"DEFAULTSPEED=10"<<endl;
  file<<"SPEED=100"<<endl;
  file<<""<<endl;
  file<<"L_1=BEGIN"<<endl;
  file<<"BYPASS=FALSE"<<endl;
  file<<"CUTTYPE=10"<<endl;
  file<<"ARRAY=0,0,0,0"<<endl;
  file<<"SPEED=1000"<<endl;
  file<<"FRAME=0,0,0,0,0,0"<<endl;
  file<<"OFFSET=0,0,0,0,0,0"<<endl;
  file<<"BLEND=1"<<endl;
  file<<"WORKPLACE=0,0"<<endl;
  for(int i=0;i<vecjumpxbegin.size();i++)
  {
    file<<"MOVEJ="<<fixed<<setprecision(3)<<vecjumpxbegin[i]<<","<<fixed<<setprecision(3)<<vecjumpybegin[i]<<","<<fixed<<setprecision(3)<<vecjumpzbegin[i]<<","<<fixed<<setprecision(3)<<vecjumprxbegin[i]<<","<<fixed<<setprecision(3)<<vecjumprybegin[i]<<","<<fixed<<setprecision(3)<<vecjumprzbegin[i]<<endl;
  }
  file<<"MOVEL=0.000,0.000,-10.000,0.000,0.000,0/30,80"<<endl;
  for(int i=0;i<storex.size();i++)
  {
    file<<"MOVEL="<<fixed<<setprecision(3)<<storex[i]<<","<<fixed<<setprecision(3)<<-storey[i]<<","<<fixed<<setprecision(3)<<storez[i]<<",0.000,0.000,0/30,80"<<endl;
  }
  file<<"MOVEJ=0.000,0.000,90.000,0.000,45.000,0/30,250"<<endl;
  for(int i=0;i<vecjumpxend.size();i++)
  {
    file<<"MOVEJ="<<fixed<<setprecision(3)<<vecjumpxend[i]<<","<<fixed<<setprecision(3)<<vecjumpyend[i]<<","<<fixed<<setprecision(3)<<vecjumpzend[i]<<","<<fixed<<setprecision(3)<<vecjumprxend[i]<<","<<fixed<<setprecision(3)<<vecjumpryend[i]<<","<<fixed<<setprecision(3)<<vecjumprzend[i]<<endl;
  }
  file<<"L_1=END"<<endl;
  file.close();
}

void TabWidget::on_pushButton_poszadd_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSZADD;
  mythread[1]->start();
}

void TabWidget::on_pushButton_posxadd_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSXADD;
  mythread[1]->start();
}

void TabWidget::on_pushButton_posxdec_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSXDEC;
  mythread[1]->start();
}

void TabWidget::on_pushButton_posyadd_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSYADD;
  mythread[1]->start();
}

void TabWidget::on_pushButton_posydec_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSYDEC;
  mythread[1]->start();
}

void TabWidget::on_pushButton_poszdec_clicked()
{
//  mythread[1]->velscale = velscale;
  mythread[1]->threadmode = POSZDEC;
  mythread[1]->start();
}



void TabWidget::on_lineEdit_stepsize_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->lineEdit_stepsize->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    stepsize = tempfloat;
    mythread[1]->stepsize = stepsize;
}

void TabWidget::on_pushButton_posalways_clicked()
{
  mythread[0]->istimer = 1;
  mythread[0]->start();
  timer->start(1000);
}
void TabWidget::getpostimer()
{


  px=mythread[0]->px;
  py=mythread[0]->py;
  pz=mythread[0]->pz;
  ow=mythread[0]->ow;
  ox=mythread[0]->ox;
  oy=mythread[0]->oy;
  oz=mythread[0]->oz;
  float j1;
  float j2;
  float j3;
  float j4;
  float j5;
  float j6;
  j1=mythread[0]->j1;
  j2=mythread[0]->j2;
  j3=mythread[0]->j3;
  j4=mythread[0]->j4;
  j5=mythread[0]->j5;
  j6=mythread[0]->j6;
  Eigen::Quaterniond q;
  q.x() = ox;
  q.y() = oy;
  q.z() = oz;
  q.w() = ow;
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  float rx=euler[2]*180/PI;
  float ry=euler[1]*180/PI;
  float rz=euler[0]*180/PI;
  QString qpx;
  qpx=QString("%1").arg(px);
  QString qpy;
  qpy=QString("%1").arg(py);
  QString qpz;
  qpz=QString("%1").arg(pz);
  QString qrx;
  qrx=QString("%1").arg(rx);
  QString qry;
  qry=QString("%1").arg(ry);
  QString qrz;
  qrz=QString("%1").arg(rz);

  ui->editor_px->setText(QString::number(px, 10, 3));
  ui->editor_py->setText(QString::number(py, 10, 3));
  ui->editor_pz->setText(QString::number(pz, 10, 3));
  ui->editor_rx->setText(QString::number(rx, 10, 3));
  ui->editor_ry->setText(QString::number(ry, 10, 3));
  ui->editor_rz->setText(QString::number(rz, 10, 3));
  ui->editor_j1->setText(QString::number(j1, 10, 3));
  ui->editor_j2->setText(QString::number(j2, 10, 3));
  ui->editor_j3->setText(QString::number(j3, 10, 3));
  ui->editor_j4->setText(QString::number(j4, 10, 3));
  ui->editor_j5->setText(QString::number(j5, 10, 3));
  ui->editor_j6->setText(QString::number(j6, 10, 3));
}

void TabWidget::on_pushButton_posshutdown_clicked()
{
    timer->stop();
    mythread[0]->terminate();//终止
    mythread[0]->wait();//使得线程阻塞等待直到退出或超时
}

void TabWidget::on_pushButton_j1add_clicked()
{
    mythread[1]->threadmode = J1ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j1dec_clicked()
{
    mythread[1]->threadmode = J1DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j2add_clicked()
{
    mythread[1]->threadmode = J2ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j2dec_clicked()
{
    mythread[1]->threadmode = J2DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j3add_clicked()
{
    mythread[1]->threadmode = J3ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j3dec_clicked()
{
    mythread[1]->threadmode = J3DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j4add_clicked()
{
    mythread[1]->threadmode = J4ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j4dec_clicked()
{
    mythread[1]->threadmode = J4DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j5add_clicked()
{
    mythread[1]->threadmode = J5ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j5dec_clicked()
{
    mythread[1]->threadmode = J5DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j6add_clicked()
{
    mythread[1]->threadmode = J6ADD;
    mythread[1]->start();
}

void TabWidget::on_pushButton_j6dec_clicked()
{
//    mythread[1]->velscale = velscale;
//    mythread[1]->stepsize = stepsize;
    mythread[1]->threadmode = J6DEC;
    mythread[1]->start();
}

void TabWidget::on_pushButton_record_clicked()
{
  moveit::planning_interface::MoveGroupInterface group("arm");

  recordpx.push_back(px);
  recordpy.push_back(py);
  recordpz.push_back(pz);
  recordow.push_back(ow);
  recordox.push_back(ox);
  recordoy.push_back(oy);
  recordoz.push_back(oz);
  QString qpx;
  qpx=QString("%1").arg(px);
  QString qpy;
  qpy=QString("%1").arg(py);
  QString qpz;
  qpz=QString("%1").arg(pz);
  QString qow;
  qow=QString("%1").arg(ow);
  QString qox;
  qox=QString("%1").arg(ox);
  QString qoy;
  qoy=QString("%1").arg(oy);
  QString qoz;
  qoz=QString("%1").arg(oz);
  QString qnumber;
  qnumber=QString("point%1").arg(recordnum+1);
  ui->tableWidget_point->insertRow(recordnum);
  ui->tableWidget_point->setItem(recordnum,0,new QTableWidgetItem(qnumber));
  ui->tableWidget_point->setItem(recordnum,1,new QTableWidgetItem(qpx));
  ui->tableWidget_point->setItem(recordnum,2,new QTableWidgetItem(qpy));
  ui->tableWidget_point->setItem(recordnum,3,new QTableWidgetItem(qpz));
  ui->tableWidget_point->setItem(recordnum,4,new QTableWidgetItem(qow));
  ui->tableWidget_point->setItem(recordnum,5,new QTableWidgetItem(qox));
  ui->tableWidget_point->setItem(recordnum,6,new QTableWidgetItem(qoy));
  ui->tableWidget_point->setItem(recordnum,7,new QTableWidgetItem(qoz));
  recordnum++;

}

void TabWidget::on_pushButton_recordclear_clicked()
{
  recordnum = 0;
  int iLen = ui->tableWidget_point->rowCount();

  for(int i=0;i<iLen;i++)
  {
   ui->tableWidget_point->removeRow(iLen-i-1);
  }
  recordpx.clear();
  recordpy.clear();
  recordpz.clear();
  recordow.clear();
  recordox.clear();
  recordoy.clear();
  recordoz.clear();

}

void TabWidget::on_editor_owgo_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_owgo->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    owgo = tempfloat;
    mythread[1]->owgo = owgo;
}

void TabWidget::on_editor_oxgo_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_oxgo->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    oxgo = tempfloat;
    mythread[1]->oxgo = oxgo;
}

void TabWidget::on_editor_oygo_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_oygo->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    oygo = tempfloat;
    mythread[1]->oygo = oygo;
}

void TabWidget::on_editor_ozgo_editingFinished()
{
  // 获取输入框内的数据
    QString temp_string = ui->editor_ozgo->text();
  // 将字符串转换成浮点数
    double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
    ozgo = tempfloat;
    mythread[1]->ozgo = ozgo;
}


void TabWidget::on_editor_velscale_editingFinished()
{
  // 获取输入框内的数据
  QString temp_string = ui->editor_velscale->text();
  // 将字符串转换成浮点数
  double tempfloat = temp_string.toFloat();
  // 保存当前的输入值
  velscale = tempfloat;
  mythread[1]->velscale = velscale;
}

void TabWidget::on_pushButton_simulation_clicked()
{
   mythread[1]->threadmode = RECORDSIMULATION;
   mythread[1]->recordpx=recordpx;
   mythread[1]->recordpy=recordpy;
   mythread[1]->recordpz=recordpz;
   mythread[1]->recordow=recordow;
   mythread[1]->recordox=recordox;
   mythread[1]->recordoy=recordoy;
   mythread[1]->recordoz=recordoz;
   mythread[1]->start();
}

void TabWidget::on_pushButton_jumpbeginadd_clicked()
{
  vecjumpxbegin.push_back(jumpx);
  vecjumpybegin.push_back(jumpy);
  vecjumpzbegin.push_back(jumpz);
  vecjumprxbegin.push_back(jumprx);
  vecjumprybegin.push_back(jumpry);
  vecjumprzbegin.push_back(jumprz);
  QString qpx;
  qpx=QString("%1").arg(jumpx);
  QString qpy;
  qpy=QString("%1").arg(jumpy);
  QString qpz;
  qpz=QString("%1").arg(jumpz);
  QString qrx;
  qrx=QString("%1").arg(jumprx);
  QString qry;
  qry=QString("%1").arg(jumpry);
  QString qrz;
  qrz=QString("%1").arg(jumprz);
  ui->tableWidget_beginjump->insertRow(jumpbeginnum);
  ui->tableWidget_beginjump->setItem(jumpbeginnum,0,new QTableWidgetItem(qpx));
  ui->tableWidget_beginjump->setItem(jumpbeginnum,1,new QTableWidgetItem(qpy));
  ui->tableWidget_beginjump->setItem(jumpbeginnum,2,new QTableWidgetItem(qpz));
  ui->tableWidget_beginjump->setItem(jumpbeginnum,3,new QTableWidgetItem(qrx));
  ui->tableWidget_beginjump->setItem(jumpbeginnum,4,new QTableWidgetItem(qry));
  ui->tableWidget_beginjump->setItem(jumpbeginnum,5,new QTableWidgetItem(qrz));
  jumpbeginnum++;
}

void TabWidget::on_pushButton_jumpbegindec_clicked()
{
  jumpbeginnum = 0;
  int iLen = ui->tableWidget_beginjump->rowCount();

  for(int i=0;i<iLen;i++)
  {
   ui->tableWidget_beginjump->removeRow(iLen-i-1);
  }
  vecjumpxbegin.clear();
  vecjumpybegin.clear();
  vecjumpzbegin.clear();
  vecjumprxbegin.clear();
  vecjumprybegin.clear();
  vecjumprzbegin.clear();
}

void TabWidget::on_pushButton_jumpendadd_clicked()
{
  vecjumpxend.push_back(jumpx);
  vecjumpyend.push_back(jumpy);
  vecjumpzend.push_back(jumpz);
  vecjumprxend.push_back(jumprx);
  vecjumpryend.push_back(jumpry);
  vecjumprzend.push_back(jumprz);
  QString qpx;
  qpx=QString("%1").arg(jumpx);
  QString qpy;
  qpy=QString("%1").arg(jumpy);
  QString qpz;
  qpz=QString("%1").arg(jumpz);
  QString qrx;
  qrx=QString("%1").arg(jumprx);
  QString qry;
  qry=QString("%1").arg(jumpry);
  QString qrz;
  qrz=QString("%1").arg(jumprz);
  ui->tableWidget_endjump->insertRow(jumpendnum);
  ui->tableWidget_endjump->setItem(jumpendnum,0,new QTableWidgetItem(qpx));
  ui->tableWidget_endjump->setItem(jumpendnum,1,new QTableWidgetItem(qpy));
  ui->tableWidget_endjump->setItem(jumpendnum,2,new QTableWidgetItem(qpz));
  ui->tableWidget_endjump->setItem(jumpendnum,3,new QTableWidgetItem(qrx));
  ui->tableWidget_endjump->setItem(jumpendnum,4,new QTableWidgetItem(qry));
  ui->tableWidget_endjump->setItem(jumpendnum,5,new QTableWidgetItem(qrz));
  jumpendnum++;
}

void TabWidget::on_pushButton_jumpenddec_clicked()
{
  jumpendnum = 0;
  int iLen = ui->tableWidget_endjump->rowCount();

  for(int i=0;i<iLen;i++)
  {
   ui->tableWidget_endjump->removeRow(iLen-i-1);
  }
  vecjumpxend.clear();
  vecjumpyend.clear();
  vecjumpzend.clear();
  vecjumprxend.clear();
  vecjumpryend.clear();
  vecjumprzend.clear();
}

void TabWidget::onRadioClickRobotMode()
{
  switch(robotmodegroup->checkedId())
     {
     case 0:
         qDebug() << QString::fromLocal8Bit("仿真模式");
         robotmode = 0;
         break;
     case 1:
         qDebug() << QString::fromLocal8Bit("离线模式");
         robotmode = 1;
         break;
     case 2:
         qDebug() << QString::fromLocal8Bit("在线模式");
         robotmode = 2;
         break;
     }

}

void TabWidget::on_pushButton_Preview_clicked()
{
//    vector<float> picturex;
//    vector<float> picturey;
//    IplImage *p_iplImg = cvLoadImage("result.jpg");

//    for (int i=0;i<p_iplImg->height;i++)
//    {
//      for (int j=0;j<(p_iplImg->width-1);j++)
//      {

//        CvScalar S0;
//        S0=cvGet2D(p_iplImg,i,j);
//        if(S0.val[0]==255)
//        {
//          //Num[i][j]=1;
//          std::cout<<"x="<<i<<" y="<<j<<std::endl;
//          picturex.push_back(i);
//          picturey.push_back(j);
//        }
//        else
//        {

//        }

//      }
//    }
//  QPainter painter(&image);
//  painter.eraseRect(QRect(0,0,800,420));
//  QPen mypen;
//  mypen.setWidth(1);                     // 1 表示点的大小（形状为方形）
//  mypen.setColor(Qt::black);
//  painter.setPen(mypen);
//  painter.drawPoint(40,40);
//  QWidget::update();

}
