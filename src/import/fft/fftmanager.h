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

#include "configurable.h"

#include "json_fwd.hpp"

#include <QObject>

#include <boost/optional.hpp>

#include <set>
#include <vector>
#include <memory>


class COMPASS;
class ConfigurationFFT;
class FFTsConfigurationDialog;
class DBFFT;

namespace dbContent {
class VariableSet;
}

class FFTManager : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void fftsChangedSignal();

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

    void configurationDialogDoneSlot();

public:
    FFTManager(const std::string& class_id, const std::string& instance_id, COMPASS* compass);
    virtual ~FFTManager();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    bool hasConfigFFT(const std::string& name);
    void createConfigFFT(const std::string& name);
    ConfigurationFFT& configFFT(const std::string& name);
    const std::vector<std::unique_ptr<ConfigurationFFT>>& configFFTs() const;
    void deleteAllFFTs();

    bool hasDBFFT(const std::string& name);
    DBFFT& dbFFT(const std::string& name);
    bool canAddNewFFTFromConfig (const std::string& name);
    void addNewFFT (const std::string& name, bool emit_signal=true); // be sure not to call from different thread
    void addNewFFT (const std::string& name, nlohmann::json info, bool emit_signal);
    const std::vector<std::unique_ptr<DBFFT>>& dbFFTs() const;

    void deleteFFT(const std::string& name);

    const std::vector<std::string>& getAllFFTNames(); // both config and db

    void saveDBFFTs();

    void exportFFTs(const std::string& filename);
    void importFFTs(const std::string& filename);

    FFTsConfigurationDialog* configurationDialog();

    std::pair<bool, float> isFromFFT(double prelim_latitute_deg, double prelim_longitude_deg,
                                     boost::optional<unsigned int> mode_s_address, bool ignore_mode_s,
                                     boost::optional<unsigned int> mode_a_code,
                                     boost::optional<float> mode_c_code);
    // returns flag, altitude in ft if true

protected:
    COMPASS& compass_;

    std::vector<std::unique_ptr<ConfigurationFFT>> config_ffts_;
    std::vector<std::unique_ptr<DBFFT>> db_ffts_;

    std::vector<std::string> fft_names_all_; // both from config and db, vector to have order

    std::unique_ptr<FFTsConfigurationDialog> config_dialog_;

    const double max_fft_plot_distance_m_ {10000}; // lat/lon distance in degress

    virtual void checkSubConfigurables();

    void loadDBFFTs();
    void sortDBFFTs();

    void updateFFTNamesAll();
};
