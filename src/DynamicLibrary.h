#pragma once

#include <string>

namespace pdal
{

class DynamicLibrary
{
public:

  static DynamicLibrary * load(const std::string & path, 
                               std::string &errorString);
  ~DynamicLibrary();
  
  void * getSymbol(const std::string & name);

private:
  DynamicLibrary();
  
  DynamicLibrary(void * handle);
  DynamicLibrary(const DynamicLibrary &);
  
private:
  void * handle_;  
};

} // namespace pdal

