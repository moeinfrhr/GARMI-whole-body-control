/** ------------------------- Revision Code History -------------------
*** Programming Language: C++
*** Description: Data Recorder
*** Released Date: Feb. 2021
*** Hamid Sadeghian
*** h.sadeghian@eng.ui.ac.ir
----------------------------------------------------------------------- */

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <array>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 10, 1> Vector10d;
typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 16, 1> Vector16d;

class Recorder {
 public:
   Recorder();
  ~Recorder();

  void setParams(double t_rec, double sampleTime, int NoDataRec, std::string name);
  void addToRec(int value);
  void addToRec(double value);
  void addToRec(double array[], int sizeofarray);
  void addToRec(std::array<double, 3> array);
  void addToRec(std::array<double, 6> array);
  void addToRec(std::array<double, 7> array);

  void addToRec(Vector3d& vector);
  void addToRec(Vector2d& vector);
  void addToRec(Vector4d& vector);
  void addToRec(Vector6d& vector);
  void addToRec(Vector7d& vector);
  void addToRec(Vector10d& vector);
  void addToRec(Vector12d& vector);
  void addToRec(Vector16d& vector);
  void addToRec(Eigen::VectorXd& vector);
  void addToRec(Eigen::Quaterniond& vector);

  void saveData();
  void next();

 private:
  int _index;
  int _columnindex;
  int _rowindex;
  double _t_rec;
  std::string _name;
  Matrix<double, Dynamic, Dynamic> _DAT;
};
