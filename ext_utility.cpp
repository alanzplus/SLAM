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

#include "ext_utility.hpp"
using namespace std;
namespace ext_utility {

istream& operator>>(istream& is, ConfigFile& config_file) {
  const auto& delimiter = config_file.delimiter();
  const auto& comment = config_file.comment();
  auto& contents = config_file.contents();
  while (!is.eof()) {
    string line;
    getline(is, line);
    if (is.eof()) break;
    // Ignore commnets
    line = line.substr(0, line.find(comment));
    size_t delimiter_idx = line.find(delimiter);
    if (delimiter_idx != string::npos) {
      string key = line.substr(0, delimiter_idx);
      string value = line.substr(delimiter_idx + 1);
      StripInPlace(key);
      StripInPlace(value);
      contents[key] = value;
    }
  }
  return is;
}

ostream& operator<<(ostream& os, const ConfigFile& config_file) {
  const auto& contents = config_file.contents();
  for (auto it = contents.begin(); it != contents.end(); ++it) {
    os << it->first << " " << config_file.delimiter()
       << it->second << endl;
  }
  return os;
}

ConfigFile::ConfigFile() : delimiter_("="), comment_("#") {}

ConfigFile::
ConfigFile(const std::string& filename,
           const std::string& delimiter,
           const std::string& comment)
 : filename_(filename), delimiter_(delimiter), comment_(comment) {
  ifstream infile(filename);
  if (!infile.is_open()) {
    cerr << "Cannot open config file:" << filename_
         << ", Exit." << endl;
    exit(-1);
  }
  // parse Config file
  infile >> (*this);
}

} // namespace internal_utility