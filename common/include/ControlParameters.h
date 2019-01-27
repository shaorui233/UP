/*! @file ControlParameters.h
 *  @brief Interface to set gains/control parameters for simulator and robot
 *  These are designed to be updated infrequently.  For high frequency data, consider using
 *  Driver Inputs or adding to the Robot Debug Data instead.
 *
 * ControlParameter: a single value, either a double, float, or s64.  Each control parameter must have a unique name
 *   Control parameters know their type as well as if they've been initialized or not
 *   ControlParameters must be initialized, either from reading from a file, reading from LCM, or some other way
 *   (TODO consider supporting Vec3's?)
 *   All control parameters must go in a ControlParameters (for now, just robot settings and sim settings)
 *
 * TODO - what should happen when this fails?
 *
 * See test_ControlParameters for an example of how this works
 */

#ifndef PROJECT_CONTROLPAREMETERS_H
#define PROJECT_CONTROLPAREMETERS_H

#include <string>
#include <map>
#include "cTypes.h"
#include "utilities.h"

#define CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH 64

enum class ControlParameterValueKind : u64 {
  FLOAT = 0,
  DOUBLE = 1,
  S64 = 2
};

std::string controlParameterValueKindToString(ControlParameterValueKind valueKind);

union ControlParameterValuePtr {
  float* f;
  double* d;
  s64* i;
};

union ControlParameterValue {
  float f;
  double d;
  s64 i;
};

std::string controlParameterValueToString(ControlParameterValue v, ControlParameterValueKind kind);

class ControlParameter;

/*!
 * ControlParameterCollections contains a map of all the control parameters.
 */
class ControlParameterCollection {
public:
  explicit ControlParameterCollection(const std::string& name) : _name(name) { }

  /*!
   * Use this to add a parameter for the first time in the RobotControlParameters or SimulatorControlParameters
   * This should only be used during initialization of a ControlParameter
   */
  void addParameter(ControlParameter* param, const std::string& name) {
    if(mapContains(_map, name)) {
      printf("[ERROR] ControlParameterCollection %s: tried to add parameter %s twice!\n", _name.c_str(), name.c_str());
      throw std::runtime_error("control parameter error");
    }
    _map[name] = param;
  }

  /*!
   * Lookup a control parameter by its name.
   * This does not modify the set field of the control parameter!
   */
  ControlParameter& lookup(const std::string& name) {
    if(mapContains(_map, name)) {
      return *_map[name];
    } else {
      // for now:
      throw std::runtime_error("parameter " + name + " wasn't found in parameter collection " + _name);
    }
  }

  std::string printToString(); //!< print all control parameters in the INI file format
  bool checkIfAllSet();        //!< are all the control parameters initialized?

  std::map<std::string, ControlParameter*> _map;

private:
  std::string _name;
};



class ControlParameter {
public:

  /*!
   * Constructors for control parameters:
   * Set the type and add to collection.
   * Doesn't "initialize" the parameter - this must be done separately.
   */
  ControlParameter(const std::string& name, double& value, ControlParameterCollection& collection, const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.d = &value;
    _kind = ControlParameterValueKind::DOUBLE;
    collection.addParameter(this, name);
  }


  ControlParameter(const std::string& name, float& value, ControlParameterCollection& collection, const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.f = &value;
    _kind = ControlParameterValueKind::FLOAT;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, s64& value, ControlParameterCollection& collection, const std::string& units = "") {
    _name = name;
    truncateName();
    _units = units;
    _value.i = &value;
    _kind = ControlParameterValueKind::S64;
    collection.addParameter(this, name);
  }

  void truncateName() {
    if(_name.length() > CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH) {
      printf("[Error] name %s is too long, shortening to ", _name.c_str());
      _name.resize(CONTROL_PARAMETER_MAXIMUM_NAME_LENGTH - 1);
      printf("%s\n", _name.c_str());
    }
  }

  /*!
   * Set initial value of the control parameter.
   * Checks to see that the types are correct
   */
  void initializeDouble(double d) {
    if(_kind != ControlParameterValueKind::DOUBLE) {
      throw std::runtime_error("Tried to initialize control parameter " + _name + " as a double!");
    }
    _set = true;
    *_value.d = d;
  }

  void initializeFloat(float f) {
    if(_kind != ControlParameterValueKind::FLOAT) {
      throw std::runtime_error("Tried to initialize control parameter " + _name + " as a float!");
    }
    _set = true;
    *_value.f = f;
  }

  void initializeInteger(s64 i) {
    if(_kind != ControlParameterValueKind::S64) {
      throw std::runtime_error("Tried to initialize control parameter " + _name + " as an integer!");
    }
    _set = true;
    *_value.i = i;
  }

  void set(ControlParameterValue value, ControlParameterValueKind kind) {
    if(kind != _kind) {
      throw std::runtime_error("type mismatch in set");
    }
    switch(kind) {
      case ControlParameterValueKind::FLOAT:
        *_value.f = value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
        *_value.d = value.d;
        break;
      case ControlParameterValueKind::S64:
        *_value.i = value.i;
        break;
      default:
        throw std::runtime_error("invalid kind");
    }
    _set = true;
  }

  ControlParameterValue get(ControlParameterValueKind kind) {
    ControlParameterValue value;
    if(kind != _kind) {
      throw std::runtime_error("type mismatch in get");
    }
    switch(_kind) {
      case ControlParameterValueKind::FLOAT:
        value.f = *_value.f;
        break;
      case ControlParameterValueKind::DOUBLE:
         value.d = *_value.d;
        break;
      case ControlParameterValueKind::S64:
        value.i = *_value.i;
        break;
      default:
        throw std::runtime_error("invalid kind");
    }
    return value;
  }

  /*!
   * Convert the value to a string that works in an INI file
   */
  std::string toString() {
    std::string result;
    switch(_kind) {
      case ControlParameterValueKind::DOUBLE:
        result += numberToString(*_value.d);
        break;
      case ControlParameterValueKind::FLOAT:
        result += numberToString(*_value.f);
        break;
      case ControlParameterValueKind::S64:
        result += std::to_string(*_value.i);
        break;
      default:
        result += "<unknown type " + std::to_string((u32)(_kind)) + "> (add it yourself in ControlParameters.h!)";
        break;
    }
    return result;
  }


  bool _set = false;
  ControlParameterValuePtr _value;
  std::string _name;
  std::string _units;
  ControlParameterValueKind _kind;
private:

};

/*!
 * Parent class for groups of parameters
 * RobotParameters and SimulatorParameters inherit from this class
 */
class ControlParameters {
public:

  /*!
   * Each control parameter group must have a unique name so the ini files don't mixed up
   * @param name
   */
  ControlParameters(const std::string& name) : collection(name), _name(name) { }


  /*!
   * If true, all parameters have been initialized in one way or another
   */
  bool isFullyInitialized() {
    return collection.checkIfAllSet();
  }

  /*!
   * Directly initialize a given control parameter
   */
  void initializeDouble(const std::string& name, double d) {
    collection.lookup(name).initializeDouble(d);
  }

  void initializeFloat(const std::string& name, float f) {
    collection.lookup(name).initializeFloat(f);
  }

  void initializeInteger(const std::string& name, s64 i) {
    collection.lookup(name).initializeInteger(i);
  }

  void writeToIniFile(const std::string& path);
  void initializeFromIniFile(const std::string& path);

  std::string generateUnitializedList();

  ControlParameterCollection collection;

protected:
  std::string _name;
};

#endif //PROJECT_CONTROLPAREMETERS_H
