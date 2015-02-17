/******************************************************************************
 * Copyright (c) 2015, Hobu Inc.
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#pragma once

#include <pdal/Dimension.hpp>
#include <string>

namespace pdal
{

struct ExtraDim
{
    std::string m_name;
    DimType m_dimType;
};

struct ExtraBytesSpec
{
    char m_reserved[2];
    uint8_t m_dataType;
    uint8_t m_options;
    char m_name[32];
    char m_reserved2[4];
    uint64_t m_noData[3]; // 24 = 3*8 bytes
    double m_min[3]; // 24 = 3*8 bytes
    double m_max[3]; // 24 = 3*8 bytes
    double m_scale[3]; // 24 = 3*8 bytes
    double m_offset[3]; // 24 = 3*8 bytes
    char m_description[32];
};

class ExtraBytesIf
{
public:
    ExtraBytesIf(const std::string& name, Dimension::Type::Enum type,
        const std::string& description)
    {
        memset(&m_extraField, 0, sizeof(m_extraField));
        setType(type);
        setName(name);
        setDescription(description);
    }


    void setType(Dimension::Type::Enum type)
    {
        using namespace Dimension::Type;

        Dimension::Type::Enum lastypes[] = {
            None, Unsigned8, Signed8, Unsigned16, Signed16,
            Unsigned32, Signed32, Unsigned64, Signed64, Float, Double
        };

        for (size_t i = 0; i < sizeof(lastypes) / sizeof(lastypes[0]); ++i)
            if (type == lastypes[i])
            {
                m_extraField.m_dataType = (uint8_t)i;
                break;
            }
    }

    void setName(std::string name)
    { 
        name.resize(32);
        memcpy(m_extraField.m_name, name.data(), 32);
    }
    void setDescription(std::string description)
    {
        description.resize(32);
        memcpy(m_extraField.m_description, description.data(), 32);
    }
    void appendTo(std::vector<uint8_t>& ebBytes)
    {
        size_t offset = ebBytes.size();
        ebBytes.resize(ebBytes.size() + sizeof(ExtraBytesSpec));
        memcpy(ebBytes.data() + offset, &m_extraField, sizeof(ExtraBytesSpec));
    }

private:
    ExtraBytesSpec m_extraField;
};

namespace LasUtils
{

inline std::vector<ExtraDim> parse(const StringList& dimString)
{
    std::vector<ExtraDim> extraDims;

    for (auto& dim : dimString)
    {
        StringList s = Utils::split2(dim, '=');
        if (s.size() != 2)
        {
            std::ostringstream oss;
            oss << "Invalid extra dimension specified: '" << dim <<
                "'.  Need <dimension>=<type>.  See documentation "
                " for details.";
            throw pdal_error(oss.str());
        }
        Utils::trim(s[0]);
        Utils::trim(s[1]);
        Dimension::Type::Enum type = Dimension::type(s[1]);
        if (type == Dimension::Type::None)
        {
            std::ostringstream oss;
            oss << "Invalid extra dimension type specified: '" <<
                dim << "'.  Need <dimension>=<type>.  See documentations "
                " for details.";
            throw pdal_error(oss.str());
        }
        ExtraDim ed;
        ed.m_name = s[0];
        ed.m_dimType.m_type = type;
        extraDims.push_back(ed);
    }
    return extraDims;
}

} // namespace LasUtils

} // namespace pdal
