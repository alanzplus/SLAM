#include "helper.hpp"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <algorithm>

using namespace std;
namespace rgbdvo {

bool Option::
GetInt(const Options& options, std::string key, int32_t* ret) {
  auto it = options.find(key);
  if (it == options.end()) {
    cerr << "Option " << key << " not specified!" << endl;
    return false;
  }
  *ret = stoi(it->second);
  return true;
}

bool Option::
GetBool(const Options& options, std::string key, bool *ret) {
  auto it = options.find(key);
  if (it == options.end()) {
    cerr << "Option " << key  << " not specified!" << endl;
    return false;
  }

  auto val_copy(it->second);
  // trim string and ignore case.
  val_copy.erase(std::remove_if(val_copy.begin(), val_copy.end(), ::isspace),
                 val_copy.end());
  // ignore case.
  std::transform(val_copy.begin(), val_copy.end(), val_copy.begin(), ::tolower);
  if (val_copy == "true") {
    *ret = true;
  } else if (val_copy == "false") {
    *ret = false;
  } else {
    cerr << "Illegal value of " << it->second
         << " for boolean option " << key << endl;
    return false;
  }
  return true;
}

bool Option::
GetDouble(const Options& options, std::string key, double *ret) {
  auto it = options.find(key);
  if (it == options.end()) {
    cerr << "Option " << key  << " not specified!" << endl;
    return false;
  }
  *ret = stod(it->second);
  return true;
}

int32_t Option::
GetIntOrDefault(const Options& options,
                std::string key,
                const Options& defaults) {
  int32_t ret;
  if (GetInt(options, key, &ret)) {
    return ret;
  }
  assert(GetInt(defaults, key, &ret));
  cerr << "Using default value of " << ret << " for option " << key << endl;
  return ret;
}

bool Option::
GetBoolOrDefault(const Options& options, std::string key,
                 const Options& defaults) {
  bool ret;
  if (GetBool(options, key, &ret)) {
    return ret;
  }

  assert(GetBool(defaults, key, &ret));
  cerr << "Using default value of "
       << (ret ? "true" : "false")
       << " for option " << key << endl;
  return ret;
}

double Option::
GetDoubleOrDefault(const Options& options, std::string key,
                   const Options& defaults) {
  double ret;
  if (GetDouble(options, key, &ret)) {
    return ret;
  }

  assert(GetDouble(defaults, key, &ret));
  cerr << "Using default value of " << ret << " for option " << key << endl;
  return ret;
}


}