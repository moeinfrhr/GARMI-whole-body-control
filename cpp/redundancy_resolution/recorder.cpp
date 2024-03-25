/** ------------------------- Revision Code History -------------------
*** Programming Language: C++
*** Description: Data Recorde
*** Released Date: Feb. 2021
*** Hamid Sadeghian
*** h.sadeghian@eng.ui.ac.ir
----------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 10, 1> Vector10d;

class Recorder {
 public:
  Recorder(double t_rec, double sampleTime, int NoDataRec, std::string name);
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
  void addToRec(Eigen::VectorXd& vector);
  void addToRec(Quaterniond& vector);

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

Recorder::Recorder(){
  _rowindex = 0;
  _columnindex = 0;
};

Recorder::Recorder(double t_rec, double sampleTime, int NoDataRec, std::string name) {
  _DAT.resize((int)(t_rec / sampleTime + 2), NoDataRec);
  _DAT.setZero();
  _t_rec = t_rec;
  _name = name;
  _rowindex = 0;
  _columnindex = 0;
};

Recorder::~Recorder() {
  saveData();
};

void Recorder::setParams(double t_rec, double sampleTime, int NoDataRec, std::string name){
  _DAT.resize((int)(t_rec / sampleTime + 2), NoDataRec);
  _DAT.setZero();
  _t_rec = t_rec;
  _name = name;
}

void Recorder::addToRec(int value) {
  _DAT(_rowindex, _columnindex) = value;
  _columnindex++;
}
void Recorder::addToRec(double value) {
  _DAT(_rowindex, _columnindex) = value;
  _columnindex++;
}
void Recorder::addToRec(double array[], int sizeofarray) {
  // cout << "TODO: size of array is manual" << endl;
  for (int i = 0; i < sizeofarray; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(std::array<double, 7> array) {
  // cout << "TODO: size of array is manual" << endl;
  for (int i = 0; i < 7; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};

void Recorder::addToRec(std::array<double, 6> array) {
  // cout << "TODO: size of array is manual" << endl;
  for (int i = 0; i < 6; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(std::array<double, 3> array) {
  // cout << "TODO: size of array is manual" << endl;
  for (int i = 0; i < 3; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector3d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector2d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector4d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector6d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector7d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector10d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Eigen::VectorXd& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Quaterniond& vector) {
  _DAT(_rowindex, _columnindex) = vector.x();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.y();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.z();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.w();
  _columnindex++;
};

/*
void Recorder::saveData() {
  std::ofstream myfile;
  myfile.open(_name + ".m");
  myfile << _name << "m" <<"=[" << _DAT << "];\n";
  myfile.close();
  cout << "\n\n\t************Data was written successfully  ************\n";
};
void Recorder::next() {
  _rowindex++;
  _columnindex = 0;
}*/

void Recorder::saveData() {
        std::ofstream myfile;
        myfile.open(_name + ".m");

        // Extract the last part of the _name string
        std::string matrixName = _name.substr(_name.find_last_of("/") + 1);

        // Write the matrix name and data to the file
        myfile << matrixName << "m =[" << _DAT << "];\n";

        myfile.close();
        std::cout << "\n\n\t************Data was written successfully  ************\n";
    }

  void Recorder::next() {
  _rowindex++;
  _columnindex = 0;
}
