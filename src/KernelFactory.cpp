/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/KernelFactory.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Utils.hpp>

#include <delta/DeltaKernel.hpp>
#include <diff/DiffKernel.hpp>
#include <info/InfoKernel.hpp>
#include <pipeline/PipelineKernel.hpp>
#include <random/RandomKernel.hpp>
#include <sort/SortKernel.hpp>
#include <translate/TranslateKernel.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <sstream>
#include <stdio.h> // for funcptr
#include <string>
#include <vector>

namespace pdal
{


void KernelFactory::loadPlugins()
{
    using namespace boost::filesystem;

    std::string driver_path("PDAL_DRIVER_PATH");
    std::string pluginDir = Utils::getenv(driver_path);

    // Only filenames that start with libpdal_plugin are candidates to be loaded
    // at runtime.  PDAL plugins are to be named in a specified form:

    // libpdal_plugin_{kerneltype}_{name}

    // For example, libpdal_plugin_kernel_ground


    // If we don't have a driver path, we're not loading anything

    if (pluginDir.size() == 0)
    {
        pluginDir = "/usr/local/lib:./lib:../lib:../bin";
    }

    std::vector<std::string> pluginPathVec;
    boost::algorithm::split(pluginPathVec, pluginDir, boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

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

            if (boost::algorithm::istarts_with(p.filename().string(), "libpdal_plugin"))
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
            // The last two tokens are the kernel type and the kernel name.
            path basename = t->first;
            path filename = t->second;

            //registerPlugin(filename.string());
            // std::string methodName = "PDALRegister_" + boost::algorithm::ireplace_first_copy(basename.string(), "libpdal_plugin_", "");
            // Utils::registerPlugin((void*)this, filename.string(), methodName);

        }
    }
}

KernelFactory::KernelFactory(bool no_plugins)
{
    PluginManager & pm = PluginManager::getInstance();
    if (!no_plugins) { pm.loadAll(PF_PluginType_Kernel); }
    PluginManager::initializePlugin(DeltaKernel_InitPlugin);
    PluginManager::initializePlugin(DiffKernel_InitPlugin);
    PluginManager::initializePlugin(InfoKernel_InitPlugin);
    PluginManager::initializePlugin(PipelineKernel_InitPlugin);
    PluginManager::initializePlugin(RandomKernel_InitPlugin);
    PluginManager::initializePlugin(SortKernel_InitPlugin);
    PluginManager::initializePlugin(TranslateKernel_InitPlugin);
}

std::unique_ptr<Kernel> KernelFactory::createKernel(std::string const& kernel_name)
{
    PluginManager & pm = PluginManager::getInstance();

    void * kernel = pm.createObject(kernel_name);
    if (!kernel)
    {
        int32_t res = pm.guessLoadByPath(kernel_name);
        if (res == 0)
            kernel = pm.createObject(kernel_name);
    }
    Kernel *k = (Kernel*)kernel;
    std::unique_ptr<Kernel> retKernel(k);
    return retKernel;
}

std::vector<std::string> KernelFactory::getKernelNames()
{
    PluginManager & pm = PluginManager::getInstance();
    PluginManager::RegistrationMap rm = pm.getRegistrationMap();
    std::vector<std::string> nv;
    for (auto r : rm)
    {
        if (r.second.pluginType == PF_PluginType_Kernel)
            nv.push_back(r.first);
    }
    return nv;
}

} // namespace pdal
