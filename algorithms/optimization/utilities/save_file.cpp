#include "save_file.hpp"

void cleaning_file(std::string _file_name, std::string & _ret_file, bool b_opt){
    if(b_opt)
        _ret_file += THIS_COM"algorithms/optimization/optimization_data/";
    else
        _ret_file += THIS_COM"algorithms/path_planning/planning_data/";

    _ret_file += _file_name;
    _ret_file += ".txt";

    std::list<string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if(gs_fileName_string.end() == iter){
        gs_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}
void create_folder(const std::string & folder_name){
    std::string full_path = THIS_COM"algorithms/optimization/optimization_data/"+folder_name;
    const int dir_err = mkdir(full_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (-1 == dir_err)
    {
        printf("Error creating directory!n");
        exit(1);
    }
}


