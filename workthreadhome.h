#ifndef WORKTHREADHOME_H
#define WORKTHREADHOME_H

#include <QThread>

class WorkThreadhome : public QThread
{
public:
  WorkThreadhome();

  void closeThread();
  int threadmode;
  double stepsize;
  bool istimer;
  float px;
  float py;
  float pz;
  float ow;
  float ox;
  float oy;
  float oz;
  float j1;
  float j2;
  float j3;
  float j4;
  float j5;
  float j6;
  double joint1go;
  double joint2go;
  double joint3go;
  double joint4go;
  double joint5go;
  double joint6go;
  double pxgo;
  double pygo;
  double pzgo;
  double owgo;
  double oxgo;
  double oygo;
  double ozgo;
  double velscale;
  std::vector <float> storex;
  std::vector <float> storey;
  std::vector <float> storez;
  std::vector <double> recordpx;
  std::vector <double> recordpy;
  std::vector <double> recordpz;
  std::vector <double> recordow;
  std::vector <double> recordox;
  std::vector <double> recordoy;
  std::vector <double> recordoz;
protected:
  virtual void run();

private:
  volatile bool isStop;       //isStop是易失性变量，需要用volatile进行申明
};

#endif // WORKTHREADHOME_H
