#include "recorder.h"

using namespace Eigen;

Recorder::Recorder() {
}

Recorder::~Recorder() {
  saveData();
}

void Recorder::setParams(double t_rec, double sampleTime, int NoDataRec, std::string name) {
  _DAT.resize((int)(t_rec / sampleTime + 2), NoDataRec);
  _DAT.setZero();
  _rowindex = 0;
  _columnindex = 0;
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
  for (int i = 0; i < sizeofarray; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
}

void Recorder::addToRec(std::array<double, 3> array) {
  for (int i = 0; i < 3; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
}

void Recorder::addToRec(std::array<double, 6> array) {
  for (int i = 0; i < 6; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
}

void Recorder::addToRec(std::array<double, 7> array) {
  for (int i = 0; i < 7; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector3d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector2d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector4d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector6d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector7d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector10d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector12d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Vector16d& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Eigen::VectorXd& vector) {
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
}

void Recorder::addToRec(Quaterniond& vector) {
  _DAT(_rowindex, _columnindex) = vector.x();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.y();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.z();
  _columnindex++;
  _DAT(_rowindex, _columnindex) = vector.w();
  _columnindex++;
}

void Recorder::saveData() {
  std::ofstream myfile;
  myfile.open(_name + ".m");

  std::string matrixName = _name.substr(_name.find_last_of("/") + 1);
  myfile << matrixName << "m =[" << _DAT << "];\n";

  myfile.close();
  std::cout << "\n\n\t************Data was written successfully  ************\n";
}

void Recorder::next() {
  _rowindex++;
  _columnindex = 0;
}
