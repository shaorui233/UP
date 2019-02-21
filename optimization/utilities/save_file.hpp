#ifndef SAVE_FILE_H
#define SAVE_FILE_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <string>
#include <list>
#include <Configuration.h>
#include <cppTypes.h>
#include <sys/stat.h>

using namespace std;

static std::list< std::string > gs_fileName_string; //global & static

void cleaning_file(std::string _file_name, std::string & _ret_file, bool b_opt);
void create_folder(const std::string & folder_name);

template<typename T>
void saveVector(const DVec<T> & _vec, std::string _name, bool b_opt = false){
    string file_name;
    cleaning_file(_name, file_name, b_opt);

    std::ofstream savefile(file_name.c_str(), ios::app);
    for (int i(0); i < _vec.rows(); ++i){
        savefile<<_vec(i)<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
}

template<typename T>
void saveVector(const std::vector<T> & _vec, std::string _name, bool b_opt = false){
    string file_name;
    cleaning_file(_name, file_name, b_opt);
    std::ofstream savefile(file_name.c_str(), ios::app);

    for (unsigned int i(0); i < _vec.size(); ++i){
        savefile<<_vec[i]<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
}

template<typename T>
void saveVector(T * _vec, std::string _name, int size, bool b_opt = false){
    string file_name;
    cleaning_file(_name, file_name, b_opt);
    std::ofstream savefile(file_name.c_str(), ios::app);
    for (int i(0); i < size; ++i){
        savefile<<_vec[i]<< "\t";
    }
    savefile<<"\n";
    savefile.flush();
}

template<typename T>
void saveValue(T _value, std::string _name, bool b_opt = false){
    string file_name;
    cleaning_file(_name, file_name, b_opt);
    std::ofstream savefile(file_name.c_str(), ios::app);

    savefile<<_value <<"\n";
    savefile.flush();
}

#endif
