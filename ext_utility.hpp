// The MIT License (MIT)

// Copyright (c) 2014.4 JZ Xuan <jzxuanuni@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// external utilities
#ifndef __EXT_UTILITY__
#define __EXT_UTILITY__
// C
#include <cstdint>
#include <cassert>
#include <cstring>
// C++
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <memory>
#include <regex>

namespace ext_utility {

#ifdef DEBUG
#define DEBUG_LOG(...) fprintf(stderr, __VA_ARGS__);
#define DUMP_VAR(var) (std::cerr << " "#var << " = " << var << std::endl)
#else
#define DEBUG_LOG(...)
#define DUMP_VAR(var)
#endif

// String Helper Methods
inline void LStripInPlace(std::string& str,
                          const std::string& delimiter = " \f\n\r\t\v") {
  str.erase(0, str.find_first_not_of(delimiter));
}

inline void RStripInPlace(std::string& str,
                          const std::string& delimiter = " \f\n\r\t\v") {
  str.erase(str.find_last_not_of(delimiter) + 1);
}

inline void StripInPlace(std::string& str,
                         const std::string& delimiter = " \f\n\r\t\v") {
  LStripInPlace(str);
  RStripInPlace(str);
}

inline std::string LStrip(std::string str,
                          const std::string& delimiter = " \f\n\r\t\v") {
  LStripInPlace(str);
  return str;
}

inline std::string RStrip(std::string str,
                          const std::string& delimiter = " \f\n\r\t\v") {
  RStripInPlace(str);
  return str;
}

inline std::string Strip(std::string str,
                         const std::string& delimiter = " \f\n\r\t\v") {
  StripInPlace(str);
  return str;
}


template<typename T>
inline std::string ToString(const T& value) {
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

template<> std::string
inline ToString<std::string>(const std::string& value) {
  return value;
}

template<class T>
inline T FromString(const std::string& str) {
  std::istringstream iss(str);
  T value;
  iss >> value;
  return value;
}

template<>
inline std::string FromString<std::string>(const std::string& str) {
  return str;
}

template<>
inline bool FromString<bool>(const std::string& str) {
  std::regex e("^(F|FALSE|N|NO|0)$", std::regex::icase);
  return std::regex_match(str, e) ? false : true;
}


class ConfigFile {
 public:
  typedef std::map<std::string, std::string> ContentType;
 public:
  ConfigFile();
  ConfigFile(const std::string& filename,
             const std::string& delimiter = "=",
             const std::string& comment = "#");
  template<class T> T Read(const std::string& key) const;
  template<class T> T Read(const std::string& key, const T& default_value) const;
  inline bool IsKeyExists(const std::string& key) const;

  inline const std::string& filename() const;
  inline const std::string& delimiter() const;
  inline const std::string& comment() const;
  inline ContentType& contents();
  inline const ContentType& contents() const;

 private:
  std::string filename_;
  std::string delimiter_;
  std::string comment_;
  ContentType contents_;
};

std::ostream& operator<<(std::ostream& os, const ConfigFile& config_file);
std::istream& operator>>(std::istream& is, ConfigFile& config_file);


inline const std::string&
ConfigFile::filename() const {
  return filename_;
}

inline const std::string&
ConfigFile::delimiter() const {
  return delimiter_;
}

inline const std::string&
ConfigFile::comment() const {
  return comment_;
}

inline ConfigFile::ContentType&
ConfigFile::contents() {
  return contents_;
}

inline const ConfigFile::ContentType&
ConfigFile::contents() const {
  return contents_;
}

template<class T> T
ConfigFile::Read(const std::string& key) const {
  if (!IsKeyExists(key)) {
    std::cerr << key << " is not exist." << std::endl;
    exit(-1);
  }
  return FromString<T>(contents_.at(key));
}

template<class T> T
ConfigFile::Read(const std::string& key, const T& default_value) const {
  if (!IsKeyExists(key)) {
    return default_value;
  }
  return FromString<T>(contents_.at(key));
}

inline bool
ConfigFile::IsKeyExists(const std::string& key) const {
  return contents_.count(key) == 1;
}

template<class T>
void CSVReader(const std::string filename,
               std::vector<std::vector<T> >& output,
               char delim = ',') {
  std::ifstream infile(filename.c_str());
  if (!infile.is_open()) {
    ;
  }

  while (!infile.eof()) {
    std::string line;
    std::getline(infile, line);
    if (infile.eof()) break;
    std::replace(line.begin(), line.end(), delim, ' ');
    if (*(line.end()-1) == ' ') {
      line = std::string(line.begin(), line.end() - 1);
    }
    std::istringstream iss(line);
    T val;
    std::vector<T> line_data;
    while (!iss.eof()) {
      iss >> val;
      line_data.push_back(val);
    }
    output.push_back(line_data);
  }
  infile.close();
}

template<class T>
void CSVWriter(const std::string filename,
               const std::vector<std::vector<T> >& input,
               char delim = ',') {
  std::ofstream outfile(filename.c_str());
  if (!outfile.is_open()) {
    ;
  }

  for (int32_t i = 0; i < input.size(); ++i) {
    for (int32_t j = 0; j < input[i].size(); ++j) {
      outfile << input[i][j] << ",";
    }
    outfile << std::endl;
  }
  outfile.close();
}

template<class T>
void IsFileOpenOrExit(const T& fs, const std::string& fn) {
  if (fs.is_open()) return;
  std::cerr << "Cannot open " << fn << ", exit." << std::endl;
  exit(-1);
}

template<class T>
size_t GetFstreamSize(T& fs) {
  fs.seekg(0, T::end);
  size_t ret = fs.tellg();
  fs.seekg(0, T::beg);
  return ret;
}

} // namespace ext_utility

#endif