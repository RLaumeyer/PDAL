#pragma once

#include <pdal/plugin.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace pdal
{

class DynamicLibrary;

/*
 * I think PluginManager can eventually be a private header, only accessible
 * through the factories, but we'll leave it as public for now.
 */
class PluginManager
{
    typedef std::map<std::string, std::shared_ptr<DynamicLibrary> > DynamicLibraryMap;
    typedef std::vector<PF_ExitFunc> ExitFuncVec;
    typedef std::vector<PF_RegisterParams> RegistrationVec;

public:
    typedef std::map<std::string, PF_RegisterParams> RegistrationMap;

    static PluginManager & getInstance();
    static int32_t initializePlugin(PF_InitFunc initFunc);
    int32_t loadAll(PF_PluginType type);
    int32_t loadAll(const std::string & pluginDirectory, PF_PluginType type);
    int32_t guessLoadByPath(const std::string & driverName);
    int32_t loadByPath(const std::string & path, PF_PluginType type);

    void * createObject(const std::string & objectType);

    int32_t shutdown();
    static int32_t registerObject(const std::string & objectType, const PF_RegisterParams * params);
    const RegistrationMap & getRegistrationMap();

private:
    ~PluginManager();
    PluginManager();
    PluginManager(const PluginManager &);

    DynamicLibrary * loadLibrary(const std::string & path, std::string & errorString);

    bool inInitializePlugin_;
    DynamicLibraryMap dynamicLibraryMap_;
    ExitFuncVec exitFuncVec_;

    RegistrationMap tempExactMatchMap_;

    RegistrationMap exactMatchMap_;
};

} // namespace pdal

