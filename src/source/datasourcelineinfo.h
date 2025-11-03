/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "json_fwd.hpp"

class DataSourceLineInfo
{
public:
    DataSourceLineInfo(const std::string& key, nlohmann::json& config);


    bool hasListenIP() const;
    const std::string listenIP() const;
    void listenIP(const std::string& value);

    const std::string mcastIP() const;
    void mcastIP(const std::string& value);

    unsigned int mcastPort() const;
    void mcastPort(unsigned int value);

    bool hasSenderIP() const;
    const std::string senderIP() const;
    void senderIP(const std::string& value);

    std::string asString() const;

private:
    const std::string key_; // L1,L2,L3,L4
    nlohmann::json& config_;

//    listen: "123.324.21.24",           # optional, kann auch "0.0.0.0" oder "any" sein
    //    mcast: "239.192.21.24:8600", # notwendig
    //    sender: "139.192.21.24" # optional, wird nur gefiltert wenn gesetzt
};
