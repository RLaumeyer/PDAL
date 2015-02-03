/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/StageFactory.hpp>

#include <pdal/Utils.hpp>
#include <pdal/PluginManager.hpp>

#include <chipper/ChipperFilter.hpp>
#include <colorization/ColorizationFilter.hpp>
#include <crop/CropFilter.hpp>
#include <decimation/DecimationFilter.hpp>
#include <ferry/FerryFilter.hpp>
#include <merge/MergeFilter.hpp>
#include <mortonorder/MortonOrderFilter.hpp>
#include <range/RangeFilter.hpp>
#include <reprojection/ReprojectionFilter.hpp>
#include <sort/SortFilter.hpp>
#include <splitter/SplitterFilter.hpp>
#include <stats/StatsFilter.hpp>
#include <transformation/TransformationFilter.hpp>

#include <pdal/BufferReader.hpp>
#include <faux/FauxReader.hpp>
#include <las/LasReader.hpp>
#include <las/LasWriter.hpp>
#include <bpf/BpfReader.hpp>
#include <bpf/BpfWriter.hpp>
#include <sbet/SbetReader.hpp>
#include <sbet/SbetWriter.hpp>
#include <qfit/QfitReader.hpp>
#include <terrasolid/TerrasolidReader.hpp>
#include <text/TextWriter.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream>
#include <string>
#include <vector>
#include <stdio.h> // for funcptr

namespace pdal
{

std::string StageFactory::inferReaderDriver(const std::string& filename)
{
    PluginManager & pm = PluginManager::getInstance();

    // filename may actually be a greyhound uri + pipelineId
    std::string http = filename.substr(0, 4);
    if (boost::iequals(http, "http") && pm.createObject("readers.greyhound"))
        return "readers.greyhound";

    std::string ext = boost::filesystem::extension(filename);
    std::map<std::string, std::string> drivers;
    drivers["las"] = "readers.las";
    drivers["laz"] = "readers.las";
    drivers["bin"] = "readers.terrasolid";
    if (pm.createObject("readers.greyhound"))
        drivers["greyhound"] = "readers.greyhound";
    drivers["qi"] = "readers.qfit";
    if (pm.createObject("readers.nitf"))
    {
        drivers["nitf"] = "readers.nitf";
        drivers["ntf"] = "readers.nitf";
        drivers["nsf"] = "readers.nitf";
    }
    drivers["bpf"] = "readers.bpf";
    drivers["sbet"] = "readers.sbet";
    drivers["icebridge"] = "readers.icebridge";
    drivers["sqlite"] = "readers.sqlite";

    if (pm.createObject("readers.rxp"))
        drivers["rxp"] = "readers.rxp";

    if (pm.createObject("readers.pcd"))
        drivers["pcd"] = "readers.pcd";

    if (ext == "") return "";
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return "";

    boost::to_lower(ext);
    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


std::string StageFactory::inferWriterDriver(const std::string& filename)
{
    PluginManager & pm = PluginManager::getInstance();

    std::string ext = boost::filesystem::extension(filename);

    boost::to_lower(ext);

    std::map<std::string, std::string> drivers;
    drivers["bpf"] = "writers.bpf";
    drivers["las"] = "writers.las";
    drivers["laz"] = "writers.las";
    if (pm.createObject("writers.pcd"))
        drivers["pcd"] = "writers.pcd";
    if (pm.createObject("writers.pclvisualizer"))
        drivers["pclviz"] = "writers.pclvisualizer";
    drivers["sbet"] = "writers.sbet";
    drivers["csv"] = "writers.text";
    drivers["json"] = "writers.text";
    drivers["xyz"] = "writers.text";
    drivers["txt"] = "writers.text";
    if (pm.createObject("writers.nitf"))
        drivers["ntf"] = "writers.nitf";
    drivers["sqlite"] = "writers.sqlite";

    if (boost::algorithm::iequals(filename, "STDOUT"))
    {
        return drivers["txt"];
    }

    if (ext == "") return drivers["txt"];
    ext = ext.substr(1, ext.length()-1);
    if (ext == "") return drivers["txt"];

    boost::to_lower(ext);
    std::string driver = drivers[ext];
    return driver; // will be "" if not found
}


pdal::Options StageFactory::inferWriterOptionsChanges(const std::string& filename)
{
    std::string ext = boost::filesystem::extension(filename);
    boost::to_lower(ext);
    Options options;

    if (boost::algorithm::iequals(ext,".laz"))
    {
        options.add("compression", true);
    }

    PluginManager & pm = PluginManager::getInstance();
    if (boost::algorithm::iequals(ext,".pcd") && pm.createObject("writers.pcd"))
    {
        options.add("format","PCD");
    }

    options.add<std::string>("filename", filename);
    return options;
}

void StageFactory::loadPlugins()
{
    using namespace boost::filesystem;

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{stagetype}_{name}

    // For example, libpdal_plugin_writer_text or libpdal_plugin_filter_color


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
        if (!boost::filesystem::is_directory(pluginPath))
            continue;
        directory_iterator dir(pluginPath), it, end;

        std::map<path, path> pluginFilenames;

        // Collect candidate filenames in the above form. Prefer symlink files
        // over hard files if their basenames are the same.
        for (it = dir; it != end; ++it)
        {
            path p = it->path();

            if (boost::algorithm::istarts_with(p.filename().string(),
                 "libpdal_plugin"))
            {
                path extension = p.extension();
                if (boost::algorithm::iends_with(extension.string(), "DLL") ||
                    boost::algorithm::iends_with(extension.string(), "DYLIB") ||
                    boost::algorithm::iends_with(extension.string(), "SO"))
                {
                    std::string basename;

                    // Step through the stems until the extension of the stem
                    // is empty. This is our basename.  For example,
                    // libpdal_plugin_writer_text.0.dylib will basename down to
                    // libpdal_plugin_writer_text and so will
                    // libpdal_plugin_writer_text.dylib
                    // copy the path so we can modify in place
                    path t = p;
                    for (; !t.extension().empty(); t = t.stem())
                    {
                        if (t.stem().extension().empty())
                        {
                            basename = t.stem().string();
                        }
                    }

                    if (pluginFilenames.find(basename) == pluginFilenames.end())
                    {
                        // We haven't already loaded a plugin with this basename,
                        // load it.
                        pluginFilenames.insert(std::pair<path, path>(basename, p));
                    }
                    else
                    {
                        // We already have a filename with the basename of this
                        // file.  If the basename of our current file is a symlink
                        // we're going to replace what's in the map with ours because
                        // we are going to presume that a symlink'd file is more
                        // cannonical than a hard file of the same name.
                        std::map<path, path>::iterator i = pluginFilenames.find(basename);
                        if (it->symlink_status().type() == symlink_file)
                        {
                            // Take the symlink over a hard SO
                            i->second = p;
                        }
                    }
                }
            }
        }

        std::map<std::string, std::string> registerMethods;

        for (std::map<path, path>::iterator t = pluginFilenames.begin();
                t!= pluginFilenames.end(); t ++)
        {
            // Basenames must be in the following form:
            // libpdal_plugin_writer_text or libpdal_plugin_filter_color
            // The last two tokens are the stage type and the stage name.
            path basename = t->first;
            path filename = t->second;

            //registerPlugin(filename.string());
            // std::string methodName = "PDALRegister_" + boost::algorithm::ireplace_first_copy(basename.string(), "libpdal_plugin_", "");
            // Utils::registerPlugin((void*)this, filename.string(), methodName);

        }
    }
}

StageFactory::StageFactory(bool no_plugins)
{
    PluginManager & pm = PluginManager::getInstance();
    if (!no_plugins)
    {
        pm.loadAll(PF_PluginType_Filter);
        pm.loadAll(PF_PluginType_Reader);
        pm.loadAll(PF_PluginType_Writer);
    }
    PluginManager::initializePlugin(FauxReader_InitPlugin);
    PluginManager::initializePlugin(LasReader_InitPlugin);
    PluginManager::initializePlugin(BpfReader_InitPlugin);
    PluginManager::initializePlugin(QfitReader_InitPlugin);
    PluginManager::initializePlugin(SbetReader_InitPlugin);
    PluginManager::initializePlugin(TerrasolidReader_InitPlugin);
    PluginManager::initializePlugin(ChipperFilter_InitPlugin);
    PluginManager::initializePlugin(ColorizationFilter_InitPlugin);
    PluginManager::initializePlugin(CropFilter_InitPlugin);
    PluginManager::initializePlugin(DecimationFilter_InitPlugin);
    PluginManager::initializePlugin(FerryFilter_InitPlugin);
    PluginManager::initializePlugin(MergeFilter_InitPlugin);
    PluginManager::initializePlugin(MortonOrderFilter_InitPlugin);
    PluginManager::initializePlugin(RangeFilter_InitPlugin);
    PluginManager::initializePlugin(ReprojectionFilter_InitPlugin);
    PluginManager::initializePlugin(SortFilter_InitPlugin);
    PluginManager::initializePlugin(SplitterFilter_InitPlugin);
    PluginManager::initializePlugin(StatsFilter_InitPlugin);
    PluginManager::initializePlugin(TransformationFilter_InitPlugin);
    PluginManager::initializePlugin(BpfWriter_InitPlugin);
    PluginManager::initializePlugin(LasWriter_InitPlugin);
    PluginManager::initializePlugin(SbetWriter_InitPlugin);
    PluginManager::initializePlugin(TextWriter_InitPlugin);
}

Stage* StageFactory::createStage2(std::string const& stage_name)
{
    PluginManager & pm = PluginManager::getInstance();

    void * stage = pm.createObject(stage_name);
    if (!stage)
    {
        int32_t res = pm.guessLoadByPath(stage_name);
        if (res == 0)
            stage = pm.createObject(stage_name);
    }

    return (Stage*)stage;
}

std::unique_ptr<Stage> StageFactory::createStage(std::string const& stage_name)
{/*
    PluginManager & pm = PluginManager::getInstance();

    void * stage = pm.createObject(stage_name);
    if (!stage)
    {
        int32_t res = pm.guessLoadByPath(stage_name);
        if (res == 0)
            stage = pm.createObject(stage_name);
    }

    Stage *s = (Stage*)stage;
  */
    std::unique_ptr<Stage> retStage(createStage2(stage_name));
    return retStage;
}

std::vector<std::string> StageFactory::getStageNames()
{
    PluginManager & pm = PluginManager::getInstance();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    std::vector<std::string> nv;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Filter ||
            r.second.pluginType == PF_PluginType_Reader ||
            r.second.pluginType == PF_PluginType_Writer)
            nv.push_back(r.first);
    }
    return nv;
}

std::map<std::string, std::string> StageFactory::getStageMap()
{
    PluginManager & pm = PluginManager::getInstance();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    std::map<std::string, std::string> sm;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Filter ||
            r.second.pluginType == PF_PluginType_Reader ||
            r.second.pluginType == PF_PluginType_Writer)
            sm.insert(std::make_pair(r.first, r.second.description));
    }
    return sm;
}


} // namespace pdal

