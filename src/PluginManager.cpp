#include <pdal/PluginManager.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <pdal/pdal_defines.h>
#include <pdal/util/FileUtils.hpp>
#include <pdal/Utils.hpp>

#include "DynamicLibrary.h"

#include <iostream> //temp
#include <memory>
#include <sstream>
#include <string>

namespace pdal
{

#if defined(PDAL_PLATFORM_OSX)
    static std::string dynamicLibraryExtension(".dylib");
#elif defined(PDAL_PLATFORM_LINUX)
    static std::string dynamicLibraryExtension(".so");
#elif defined(PDAL_PLATFORM_WIN32)
    static std::string dynamicLibraryExtension(".dll");
#endif

static bool isValid(const PF_RegisterParams * params)
{
    if (!params || !params->createFunc || !params->destroyFunc)
        return false;

    return true;
}

int32_t PluginManager::registerObject(const std::string & objectType, const PF_RegisterParams * params)
{
    if (!isValid(params))
        return -1;

    PluginManager & pm = PluginManager::getInstance();

    // skipped version check for now

    if (pm.exactMatchMap_.find(objectType) != pm.exactMatchMap_.end())
        return -1;

    pm.exactMatchMap_[objectType] = *params;
    return 0;
}

PluginManager & PluginManager::getInstance()
{
    static PluginManager instance;
    return instance;
}

int32_t PluginManager::loadAll(PF_PluginType type)
{
    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // If we don't have a driver path, we'll default to /usr/local/lib and lib

    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH << ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir,
        boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        loadAll(pluginPath, type);
    }

    return 0;
}

int32_t PluginManager::loadAll(const std::string & pluginDirectory, PF_PluginType type)
{
    if (pluginDirectory.empty())
        return -1;

    if (!boost::filesystem::exists(pluginDirectory) || !boost::filesystem::is_directory(pluginDirectory))
        return -1;

    boost::filesystem::directory_iterator dir(pluginDirectory);
    for (auto const& it : dir)
    {
        boost::filesystem::path full_path = it.path();

        if (boost::filesystem::is_directory(full_path))
            continue;

        std::string ext = full_path.extension().string();
        if (ext != dynamicLibraryExtension)
            continue;

        loadByPath(full_path.string(), type);
    }

    return 0;
}

int32_t PluginManager::initializePlugin(PF_InitFunc initFunc)
{
    PluginManager & pm = PluginManager::getInstance();

    PF_ExitFunc exitFunc = initFunc();
    if (!exitFunc)
        return -1;

    pm.exitFuncVec_.push_back(exitFunc);
    return 0;
}

PluginManager::PluginManager() : inInitializePlugin_(false)
{}

PluginManager::~PluginManager()
{
    shutdown();
}

int32_t PluginManager::shutdown()
{
    int32_t result = 0;
    for (auto const& func : exitFuncVec_)
    {
        try
        {
            result = (*func)();
        }
        catch (...)
        {
            result = 01;
        }
    }

    dynamicLibraryMap_.clear();
    exactMatchMap_.clear();
    exitFuncVec_.clear();

    return result;
}

int32_t PluginManager::guessLoadByPath(const std::string& driverName)
{
    // parse the driver name into an expected pluginName, e.g., writers.las => libpdal_plugin_writer_las
    std::vector<std::string> driverNameVec;
    boost::algorithm::split(driverNameVec, driverName,
        boost::algorithm::is_any_of("."), boost::algorithm::token_compress_on);

    std::string pluginName = "libpdal_plugin_" + driverNameVec[0] + "_" + driverNameVec[1];

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // If we don't have a driver path, we'll default to /usr/local/lib and lib

    if (pluginDir.size() == 0)
    {
        std::ostringstream oss;
        oss << PDAL_PLUGIN_INSTALL_PATH << ":/usr/local/lib:./lib:../lib:../bin";
        pluginDir = oss.str();
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir,
        boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (const auto& pluginPath : pluginPathVec)
    {
        boost::filesystem::path path(pluginPath);

        if (path.empty())
            continue;

        if (!boost::filesystem::exists(path) || !boost::filesystem::is_directory(path))
            continue;

        boost::filesystem::directory_iterator dir(path);
        for (auto const& it : dir)
        {
            boost::filesystem::path full_path = it.path();
            
            if (boost::filesystem::is_directory(full_path))
                continue;

            std::string ext = full_path.extension().string();
            if (ext != dynamicLibraryExtension)
                continue;

            PF_PluginType type;
            if (driverNameVec[0] == "readers")
                type = PF_PluginType_Reader;
            else if (driverNameVec[0] == "kernels")
                type = PF_PluginType_Kernel;
            else if (driverNameVec[0] == "filters")
                type = PF_PluginType_Filter;
            else if (driverNameVec[0] == "writers")
                type = PF_PluginType_Writer;

            loadByPath(full_path.string(), type);
        }
   }

    return 0;
}

int32_t PluginManager::loadByPath(const std::string& pluginPath, PF_PluginType type)
{
    boost::filesystem::path path(pluginPath);

    bool isValid = false;
    if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_kernel"))
    {
        if (type == PF_PluginType_Kernel) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_filter"))
    {
        if (type == PF_PluginType_Filter) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_reader"))
    {
        if (type == PF_PluginType_Reader) { isValid = true; }
    }
    else if (boost::algorithm::istarts_with(path.filename().string(), "libpdal_plugin_writer"))
    {
        if (type == PF_PluginType_Writer) { isValid = true; }
    }

    if (!isValid)
        return -1;

/*  // I'm punting on the possibility of symlinks for now...
    #ifndef WIN32
    if (path.isSymbolicLink())
    {
        char buff[APR_PATH_MAX+1];
        int length = ::readlink(path, buff, APR_PATH_MAX);
        if (length < 0)
            return -1;

        path = std::string(buff, length);
    }
    #endif
*/
    if (dynamicLibraryMap_.find(path.string()) != dynamicLibraryMap_.end())
        return -1;

    //path.makeAbsolute();

    std::string errorString;
    DynamicLibrary * d = loadLibrary(boost::filesystem::complete(path).string(), errorString);
    if (!d)
        return -1;

    PF_InitFunc initFunc = (PF_InitFunc)(d->getSymbol("PF_initPlugin"));
    if (!initFunc)
        return -1;

    int32_t res = initializePlugin(initFunc);
    if (res < 0)
        return res;

    return 0;
}

void * PluginManager::createObject(const std::string & objectType)
{
    if (exactMatchMap_.find(objectType) != exactMatchMap_.end())
    {
        PF_RegisterParams & rp = exactMatchMap_[objectType];
        void * object = rp.createFunc();
        if (object)
        {
            return object;
        }
    }

    return NULL;
}

DynamicLibrary * PluginManager::loadLibrary(const std::string & path, std::string & errorString)
{
    DynamicLibrary * d = DynamicLibrary::load(path, errorString);
    if (!d)
        return NULL;

    dynamicLibraryMap_[boost::filesystem::complete(path).string()] = std::shared_ptr<DynamicLibrary>(d);
    return d;
}

const PluginManager::RegistrationMap & PluginManager::getRegistrationMap()
{
    return exactMatchMap_;
}

} // namespace pdal

