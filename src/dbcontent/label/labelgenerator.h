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

#include "labelgeneratorconfig.h"
#include "json_fwd.hpp"
#include "labeldirection.h"

//#include <osg/Matrixd>

#include <QObject>

#include <set>

class Buffer;
class DBContentManager;
class GeometryLeafItemLabels;

namespace dbContent
{

class LabelContentDialog;
class VariableSet;

/**
*/
class LabelGenerator : public QObject
{
    Q_OBJECT

signals:
    void labelConfigChanged();
    void labelOptionsChangedSignal();
    void labelLinesChangedSignal();
    void labelClearAllSignal();
    //void configChanged();

public slots:
    void editLabelContentsDoneSlot();

public:
    LabelGenerator(DBContentManager& manager);
    virtual ~LabelGenerator();

    std::vector<std::string> getLabelTexts(const std::string& dbcontent_name, unsigned int buffer_index);
    std::vector<std::string> getFullTexts(const std::string& dbcontent_name, unsigned int buffer_index);

    bool autoLabel() const;
    void autoLabel(bool auto_label);

    void autoAdustCurrentLOD(unsigned int num_labels_on_screen);

    unsigned int currentLOD() const;
    void currentLOD(unsigned int current_lod);

    bool autoLOD() const;
    void autoLOD(bool auto_lod);

    float labelDistance() const;
    void labelDistance(float label_distance);

    void toggleUseUTN();
    bool useUTN();

    void addLabelDSID(unsigned int ds_id);
    void removeLabelDSID(unsigned int ds_id);
    void labelAllDSIDs();
    void labelNoDSIDs();
    bool anyDSIDLabelWanted();
    bool labelWanted(unsigned int ds_id);
    bool labelWanted(std::shared_ptr<Buffer> buffer, unsigned int index);

    bool filterMode3aActive() const;
    void filterMode3aActive(bool filter_active);
    std::string filterMode3aValues() const;
    void filterMode3aValues(const std::string& filter_values);

    bool filterTIActive() const;
    void filterTIActive(bool filter_active);
    std::string filterTIValues() const;
    void filterTIValues(const std::string& filter_values);

    bool filterTAActive() const;
    void filterTAActive(bool filter_active);
    std::string filterTAValues() const;
    void filterTAValues(const std::string& filter_values);

    bool filterModecMinActive() const;
    void filterModecMinActive(bool value);

    float filterModecMinValue() const;
    void filterModecMinValue(float value);

    bool filterModecMaxActive() const;
    void filterModecMaxActive(bool value);

    float filterModecMaxValue() const;
    void filterModecMaxValue(float value);

    bool filterModecNullWanted() const;
    void filterModecNullWanted(bool value);

    LabelDirection labelDirection (unsigned int ds_id);
    float labelDirectionAngle (unsigned int ds_id);
    void labelDirection (unsigned int ds_id, LabelDirection direction);

    unsigned int labelLine (unsigned int ds_id); // returns 0...3
    void labelLine (unsigned int ds_id, unsigned int line);
    void updateAvailableLabelLines(); // updates lines to be label according to available lines with loaded data

    void editLabelContents(const std::string& dbcontent_name);

    void checkLabelConfig();
    nlohmann::json labelConfig() const;

    void addVariables (const std::string& dbcontent_name, dbContent::VariableSet& read_set);

    bool declutterLabels() const;
    void declutterLabels(bool declutter_labels);

    bool showDeclutteringInfoOnce() const;
    void showDeclutteringInfoOnce(bool value);

    unsigned int maxDeclutterlabels() const;

    bool filterPrimaryOnlyActive() const;
    void filterPrimaryOnlyActive(bool value);

    float labelOpacity() const;
    void labelOpacity(float label_opacity);

    LabelGeneratorConfig& config() { return config_; }
    void updateFilterValuesFromStrings();

protected:
    //virtual void checkSubConfigurables();

    bool updateM3AValuesFromStr(const std::string& values);
    bool updateTIValuesFromStr(const std::string& values);
    bool updateTAValuesFromStr(const std::string& values);

    std::string getVariableName(const std::string& dbcontent_name, unsigned int key);
    std::string getVariableValue(const std::string& dbcontent_name, unsigned int key,
                                 std::shared_ptr<Buffer>& buffer, unsigned int index, bool long_val=false);
    std::string getVariableUnit(const std::string& dbcontent_name, unsigned int key);

    std::string getMode3AText (const std::string& dbcontent_name,
                               unsigned int buffer_index, std::shared_ptr<Buffer>& buffer);
    std::string getModeCText (const std::string& dbcontent_name,
                              unsigned int buffer_index, std::shared_ptr<Buffer>& buffer);

    //void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    DBContentManager& dbcont_manager_;

    LabelGeneratorConfig config_;

//    bool  auto_lod_    {true};
//    float current_lod_ {1}; // 1, 2, 3, float for filter function

    bool show_decluttering_info_once_ {false};

    float label_distance_ {0.5}; // 0 ... 1

    std::set<unsigned int> filter_m3a_values_set_; // dec
    bool filter_m3a_null_wanted_ {false};
    std::set<std::string> filter_ti_values_set_;
    bool filter_ti_null_wanted_ {false};
    std::set<unsigned int> filter_ta_values_set_; // dec
    bool filter_ta_null_wanted_ {false};

    //std::set<GeometryLeafItemLabels*> item_labels_;
    std::unique_ptr<LabelContentDialog> label_edit_dialog_;
};

}
