#include "include/Utilities/save_file.h"

void cleaning_file(
        const std::string & folder_name, 
        const std::string & file_name, std::string & full_ret_file){
    
    full_ret_file = THIS_COM + folder_name + file_name;
    full_ret_file += ".txt";

    std::list<std::string>::iterator iter = 
        std::find(gs_fileName_string.begin(), gs_fileName_string.end(), full_ret_file);
    if(gs_fileName_string.end() == iter){
        gs_fileName_string.push_back(full_ret_file);
        remove(full_ret_file.c_str());
    }
}
void create_folder(const std::string & folder_name){
    std::string full_path = THIS_COM + folder_name;
    const int dir_err = mkdir(full_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (-1 == dir_err)
    {
        std::cout<<full_path<<std::endl;
        printf("Error creating directory!\n");
        exit(1);
    }
}


