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

#include "fft/configurationfft.h"
#include "fft/dbfft.h"
#include "fftmanager.h"
#include "logger.h"

using namespace std;
using namespace nlohmann;


ConfigurationFFT::ConfigurationFFT(const std::string& class_id, const std::string& instance_id,
                                                 FFTManager& ds_manager)
    : Configurable(class_id, instance_id, &ds_manager)
{
    registerParameter("name", &name_, std::string());
    registerParameter("info", &info_, {});
}

ConfigurationFFT::~ConfigurationFFT()
{
}

DBFFT* ConfigurationFFT::getAsNewDBFFT()
{
    DBFFT* new_fft = new DBFFT();
    new_fft->name(name_);

    if (!info_.is_null())
    {
        traced_assert(info_.is_object());
        new_fft->info(info_);
    }

    loginf << "name " << new_fft->name()
            << " info '" << new_fft->info().dump() << "'";

    return new_fft;
}

