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

#include "fftbase.h"
#include "configurable.h"

class FFTManager;

class DBFFT;

class ConfigurationFFT : public Configurable, public FFTBase
{
public:
    /// @brief Constructor backed by a json reference
    ConfigurationFFT(nlohmann::json& config, FFTManager* parent);
    virtual ~ConfigurationFFT();

    DBFFT* getAsNewDBFFT();

protected:
    virtual void checkSubConfigurables() {}
};

