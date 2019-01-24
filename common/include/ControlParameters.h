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
#include <unordered_map>
#include "cTypes.h"
#include "utilities.h"


enum class ControlParameterValueKind : u64 {
  FLOAT = 0,
  DOUBLE = 1,
  S64 = 2
};

union ControlParameterValue {
  float* f;
  double* d;
  s64* i;
};


class ControlParameter;

/*!
 * ControlParameterCollections contains a unordered_map of all the control parameters.
 */
class ControlParameterCollection {
public:
  explicit ControlParameterCollection(const std::string& name) : _name(name) { }

  /*!
   * Use this to add a parameter for the first time in the RobotControlParameters or SimulatorControlParameters
   * This should only be used during initialization of a ControlParameter
   */
  void addParameter(ControlParameter* param, const std::string& name) {
    if(uMapContains(_map, name)) {
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
    if(uMapContains(_map, name)) {
      return *_map[name];
    } else {
      // for now:
      throw std::runtime_error("parameter " + name + " wasn't found in parameter collection " + _name);
    }
  }

  std::string printToString(); //!< print all control parameters in the INI file format
  bool checkIfAllSet();        //!< are all the control parameters initialized?

  std::unordered_map<std::string, ControlParameter*> _map;

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
  ControlParameter(const std::string& name, double& value, ControlParameterCollection& collection) {
    _name = name;
    _value.d = &value;
    _kind = ControlParameterValueKind::DOUBLE;
    collection.addParameter(this, name);
  }


  ControlParameter(const std::string& name, float& value, ControlParameterCollection& collection) {
    _name = name;
    _value.f = &value;
    _kind = ControlParameterValueKind::FLOAT;
    collection.addParameter(this, name);
  }

  ControlParameter(const std::string& name, s64& value, ControlParameterCollection& collection) {
    _name = name;
    _value.i = &value;
    _kind = ControlParameterValueKind::S64;
    collection.addParameter(this, name);
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


  bool _set = false;
  ControlParameterValue _value;
  std::string _name;
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
