//customizedPrint.h
#ifndef _CUSTOMIZEDPRINT_H_
#define _CUSTOMIZEDPRINT_H_

#include <iostream>
#include <string>
#include <iomanip>

void printTimeEllapse(const double& start_time, const double& end_time, const char* c){
	std::cout << c << ": " << (end_time - start_time)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;
}

void printMessage(const char* c){
	std::cout << "[" << std::clock() << "] " << c << std::endl;
}

template <typename T>
void printVector(const T* const v, const int& size){
	for (int i = 0; i<size; i++){
      std::cout << std::setw(5) << std::setprecision(-1) << v[i];
      if (i < size - 1) {
      	std::cout << ",";
      } 
    }
    std::cout << ";" << std::endl;
}

template <typename T>
void printMatrix(const T* const M, const int& size_X, const int& size_Y){
	for (int i = 0; i<size_X; i++){
		for (int j = 0; j<size_Y; j++){
			std::cout << std::setw(5) << std::setprecision(-1) << M[i][j] << ",";
		} 
		std::cout << std::endl;
    }    
}

#endif