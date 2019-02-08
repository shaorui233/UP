#ifndef PARAMETER_HANDLER
#define PARAMETER_HANDLER

#include <dynacore_yaml-cpp/yaml.h>
#include <string>
#include <vector>

class ParamHandler {
public:
  ParamHandler(const std::string &file_name);

  virtual ~ParamHandler();

  bool getString(const std::string &key, std::string &str_value);

  template<typename T>
  bool getVector(const std::string &key, std::vector<T> &vec_value) {
    vec_value = config_[key].as<std::vector<T> >();
    return true;
  }

  template<typename T>
  bool getVector(const std::string &category, const std::string &key, std::vector<T> &vec_value) {
    vec_value = config_[category][key].as<std::vector<T>>();
    return true;
  }


  template<typename T>
  bool getValue(const std::string &key, T &T_value) {
    T_value = config_[key].as<T>();
    return true;
  }

  template<typename T>
  bool getValue(const std::string& category, const std::string &key, T &T_value) {
    T_value = config_[category][key].as<T>();
    return true;
  }


  bool getBoolean(const std::string &key, bool &bool_value);

  bool getInteger(const std::string &key, int &int_value);

protected:
  dynacore_YAML::Node config_;
};

#endif
