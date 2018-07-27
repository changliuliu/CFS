#ifndef _LOADPARAMETER_H_
#define _LOADPARAMETER_H_
// basic file operations
#include <iostream>
#include <fstream>
using namespace std;

int loadParameter(std::vector<int>& par){
  fstream myfile;
  par.clear();
  myfile.open ("parameter/parameter.txt");
  int data;
  while (!myfile.eof()){
  	myfile >> data;
  	par.push_back(data);
  	std::cout << data << endl;
  }
  myfile.close();
  return 0;
}

int loadAlgorithm(){
  fstream myfile;
  myfile.open ("parameter/algorithm.txt");
  int data;
  myfile >> data;
  myfile.close();
  return data;
}

int loadMaxIt(){
  fstream myfile;
  myfile.open ("parameter/algorithm.txt");
  int data;
  myfile >> data >> data;
  myfile.close();
  return data;
}

#endif