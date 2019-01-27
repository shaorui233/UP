/*! @file ControlParameters.cpp
 *  @brief Interface to set gains/control parameters for simulator and robot
 *  These are designed to be updated infrequently.  For high frequency data, consider using
 *  Driver Inputs or adding to the Robot Debug Data instead.
 */

#include "ControlParameters.h"
#include "utilities.h"
#include "INIReader.h"

#include <utility>

std::string controlParameterValueKindToString(ControlParameterValueKind valueKind) {
  switch(valueKind) {
    case ControlParameterValueKind::S64:
      return "s64";
    case ControlParameterValueKind::DOUBLE:
      return "double";
    case ControlParameterValueKind::FLOAT:
      return "float";
    default:
      return "unknown-ControlParameterValueKind";
  }
}

std::string controlParameterValueToString(ControlParameterValue value, ControlParameterValueKind kind) {
  std::string result;
  switch(kind) {
    case ControlParameterValueKind::DOUBLE:
      result += numberToString(value.d);
      break;
    case ControlParameterValueKind::FLOAT:
      result += numberToString(value.f);
      break;
    case ControlParameterValueKind::S64:
      result += std::to_string(value.i);
      break;
    default:
      result += "<unknown type " + std::to_string((u32)(kind)) + "> (add it yourself in ControlParameterInterface.h!)";
      break;
  }
  return result;
}

bool ControlParameterCollection::checkIfAllSet() {
  for(auto& kv : _map) {
    if(!kv.second->_set) {
      return false;
    }
  }
  return true;
}

std::string ControlParameters::generateUnitializedList() {
  std::string result;
  for(auto& kv : collection._map) {
    if(!kv.second->_set) {
      result += kv.second->_name + " (" + kv.first + ")\n";
    }
  }

  return result;
}

std::string ControlParameterCollection::printToString() {
  std::string result = ";; Generated on " + getCurrentTimeAndDate() + "\n";

  // ini section
  result += "[";
  result += _name + "]\n\n";

  std::vector<std::string> lines;

  // names (and max name length)
  int maxLength_name = 0;
  for(auto& kv : _map) {
    maxLength_name = std::max(maxLength_name, (int)kv.first.length());
    lines.push_back(kv.first);
  }

  // pad, equals sign, and number
  size_t i = 0;
  int maxLength_number = 0;
  for(auto& kv : _map) {
    int charsToAdd = maxLength_name - (int)lines[i].length();
    assert(charsToAdd >= 0);
    for(int j = 0; j < charsToAdd; j++) {
      lines[i].push_back(' ');
    }
    lines[i] += " = ";
    lines[i] += kv.second->toString();
    maxLength_number = std::max(maxLength_number, (int)lines[i].length());
    i++;
  }

  // pad, ;;, and units
  i = 0;
  for(auto& kv : _map) {
    int charsToAdd = maxLength_number - (int)lines[i].length();
    assert(charsToAdd >= 0);
    for(int j = 0; j < charsToAdd; j++) {
      lines[i].push_back(' ');
    }
    lines[i] += " ;; ";
    if(kv.second->_units.empty()) {
      lines[i] += "No units specified.  Add them!";
    } else {
      lines[i] += kv.second->_units;
    }

    i++;
  }

  // combine lines
  for(auto& line : lines) {
    result += line + "\n";
  }

  return result;
}


void ControlParameters::writeToIniFile(const std::string &path) {
  writeStringToFile(path, collection.printToString());
}

void ControlParameters::initializeFromIniFile(const std::string &path) {
  INIReader iniReader(path);
  if(iniReader.ParseError() < 0) {
    printf("[ERROR] Could not open ini file %s : not initializing control parameters!\n", path.c_str());
    throw std::runtime_error("ini file bad");
  }

  std::set<std::string> sections = iniReader.GetSections();

  if(sections.size() != 1) {
    printf("[ERROR] INI file %s had %ld sections (expected 1) : not initializing control parameters\n",
            path.c_str(), sections.size());
    throw std::runtime_error("ini file bad");
  }

  std::string sectionName = *(sections.begin());

  if(sectionName != _name) {
    printf("[ERROR] INI file %s has section name %s, which cannot be used to initialize %s\n", path.c_str(), sectionName.c_str(), _name.c_str());
    throw std::runtime_error("ini file bad");
  }

  std::set<std::string> parameterNames = iniReader.GetFields(sectionName);

  for(auto& name : parameterNames) {
    ControlParameter& cp = collection.lookup(name);
    switch(cp._kind) {
      case ControlParameterValueKind::DOUBLE:
        cp.initializeDouble(iniReader.GetReal(sectionName, name, 0.));
        break;
      case ControlParameterValueKind::FLOAT:
        cp.initializeFloat((float)iniReader.GetReal(sectionName, name, 0.));
        break;
      case ControlParameterValueKind::S64:
        cp.initializeInteger(iniReader.GetInteger(sectionName, name, 0));
        break;
      default:
        throw std::runtime_error("can't read type " + std::to_string((u32)cp._kind) + " from ini file");
        break;
    }
  }
}