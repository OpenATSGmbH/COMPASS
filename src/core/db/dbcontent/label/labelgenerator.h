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
#include "labelsettings.h"
#include "labelplacement_defs.h"

//#include <osg/Matrixd>

#include <QObject>

#include <cstdint>
#include <map>
#include <optional>
#include <set>

class Buffer;
class DBContentManager;
class GeometryLeafItemLabels;

template <class T>
class NullableVector;

namespace dbContent
{

class Variable;
class VariableSet;

/**
*/
class LabelGenerator : public QObject, public LabelSettings
{
    Q_OBJECT

signals:
    void labelConfigChanged();
    void labelOptionsChangedSignal();
    void labelContentsChangedSignal(); // label texts of existing labels need to be regenerated
    void labelLinesChangedSignal();
    void labelClearAllSignal();
    void labelOpacityChangedSignal(); // only the label display needs a refresh
    //void configChanged();

public:
    /**
     * Resolved columns of one buffer for evaluating the label filters of many of its rows,
     * avoids the name based variable and column lookups per row.
     * Notice: Only valid while the buffer is unchanged => build it per pass, never store it.
     */
    struct FilterContext
    {
        const NullableVector<unsigned int>*  line_vec   {nullptr};
        const NullableVector<unsigned int>*  ds_id_vec  {nullptr};
        const NullableVector<unsigned int>*  m3a_vec    {nullptr}; // mode 3/a filter or primary only
        const NullableVector<float>*         mc_vec     {nullptr}; // mode c filter or primary only
        const NullableVector<std::string>*   acid_vec   {nullptr}; // ti filter or primary only
        const NullableVector<std::string>*   cs_fpl_vec {nullptr}; // ti filter, cat062 only
        const NullableVector<unsigned int>*  acad_vec   {nullptr}; // ta filter or primary only
        const NullableVector<unsigned char>* type_vec   {nullptr}; // primary only

        // variable of an active filter not available in the dbcontent => rows fail that filter
        bool m3a_missing  {false};
        bool mc_missing   {false};
        bool acid_missing {false};
        bool acad_missing {false};
    };

    LabelGenerator(DBContentManager& manager);
    virtual ~LabelGenerator();

    // the caller passes the buffer its indices refer into (e.g. the geometry
    // layer's own buffer) - never resolved via the current view dataset, which
    // may already have been swapped while old geometry is still displayed
    std::vector<std::string> getLabelTexts(const std::string& dbcontent_name,
                                           const std::shared_ptr<Buffer>& buffer,
                                           unsigned int buffer_index,
                                           const std::optional<unsigned int>& override_lod = std::optional<unsigned int>());
    std::vector<std::string> getFullTexts(const std::string& dbcontent_name,
                                          const std::shared_ptr<Buffer>& buffer,
                                          unsigned int buffer_index);

    bool autoLabel() const;
    void autoLabel(bool auto_label);

    void autoAdustCurrentLOD(unsigned int num_labels_on_screen);

    unsigned int currentLOD() const override;
    void currentLOD(unsigned int current_lod);
    void fixedLOD(unsigned int lod); // switches auto lod off and selects the given lod

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
    FilterContext filterContext(const std::shared_ptr<Buffer>& buffer);
    bool labelWanted(const FilterContext& context, unsigned int index);

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

    LabelDirection labelDirection (unsigned int ds_id) override;
    float labelDirectionAngle (unsigned int ds_id);
    void labelDirection (unsigned int ds_id, LabelDirection direction);

    static float labelDirectionAngle(LabelDirection direction);

    unsigned int labelLine (unsigned int ds_id); // returns 0...3
    void labelLine (unsigned int ds_id, unsigned int line);
    void updateAvailableLabelLines(); // updates lines to be label according to available lines with loaded data

    void invalidateDSLabelSettings(); // to be called if the per data source config was changed from outside

    // increased on every change of the label config, lets derived caches (e.g. label filter
    // results) detect that they are outdated
    uint64_t configGeneration() const { return config_generation_; }

    void editLabelContents(const std::string& dbcontent_name);

    void checkLabelConfig();
    nlohmann::json labelConfig() const;

    void addVariables (const std::string& dbcontent_name, dbContent::VariableSet& read_set);
    bool labelVariablesLoaded(); // checks if all label variables are contained in the current buffers

    /// the variable a label definition names: of the data content, or a Report Variable joined
    /// onto it, see readme_dynamic_dbcontent.md Section 4.4. Null if not available
    dbContent::Variable* labelVariable(const std::string& dbcontent_name, const std::string& var_name);

    bool declutterLabels() const;
    void declutterLabels(bool declutter_labels);

    bool declutterLabelsSmart() const;
    void declutterLabelsSmart(bool declutter_labels_smart);

    label_placement::Method labelPlacementMethod() const;
    void labelPlacementMethod(label_placement::Method method);

    bool showDeclutteringInfoOnce() const;
    void showDeclutteringInfoOnce(bool value);

    unsigned int maxDeclutterLabels() const;

    bool filterPrimaryOnlyActive() const;
    void filterPrimaryOnlyActive(bool value);

    float labelOpacity() const override;
    void labelOpacity(float label_opacity);

    // view point defined labels: rec_num to label level (LOD 1-3), set when a view
    // point with a "labels" key is shown, cleared when it is unshown
    void setViewPointLabels(const std::map<unsigned long, unsigned int>& labels);
    void clearViewPointLabels();
    bool hasViewPointLabels() const { return !view_point_labels_.empty(); }
    const std::map<unsigned long, unsigned int>& viewPointLabels() const override { return view_point_labels_; }

    LabelGeneratorConfig& config() { return config_; }
    void updateFilterValuesFromStrings();

    DBContentManager& dbContentManager() { return dbcont_manager_; }

    static const int          LabelAnchorLengthPxDefault;
    static const unsigned int MaxDeclutterLabels;               // no decluttering above this number of visible labels
    static const unsigned int MaxDeclutterLabelsConstraintBased; // the same for the constraint based method
    static const float        MaxDeclutterScreenRatio;           // no decluttering if the labels cover more of the screen
                                                                 // (0 = no limit)

protected:
    //virtual void checkSubConfigurables();

    bool setLabelLine (unsigned int ds_id, unsigned int line);

    bool updateM3AValuesFromStr(const std::string& values);
    bool updateTIValuesFromStr(const std::string& values);
    bool updateTAValuesFromStr(const std::string& values);

    std::string getVariableName(const std::string& dbcontent_name, unsigned int key);
    std::string getVariableValue(const std::string& dbcontent_name, unsigned int key,
                                 const std::shared_ptr<Buffer>& buffer, unsigned int index, bool long_val=false);
    std::string getVariableUnit(const std::string& dbcontent_name, unsigned int key);

    std::string getMode3AText (const std::string& dbcontent_name,
                               unsigned int buffer_index, const std::shared_ptr<Buffer>& buffer);

    // Classifies a raw ACID into the space-stripped usable_id (if specific) or
    // fallback_id (if unspecific, e.g. "00000000"), without overwriting an
    // already-filled slot. Lets labels demote unspecific ACIDs below ACAD/Mode 3/A.
    void classifyIdentification(const std::string& raw_acid,
                                std::string& usable_id, std::string& fallback_id) const;
    std::string getModeCText (const std::string& dbcontent_name,
                              unsigned int buffer_index, const std::shared_ptr<Buffer>& buffer);

    //void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    DBContentManager& dbcont_manager_;

    LabelGeneratorConfig config_;

//    bool  auto_lod_    {true};
//    float current_lod_ {1}; // 1, 2, 3, float for filter function

    bool show_decluttering_info_once_ {false};

    label_placement::Method label_placement_method_ {label_placement::Method::ConstraintBased}; // not part of the config

    float auto_lod_value_ {1}; // runtime lod adjusted by auto lod, not part of the config (1...3, float for filter function)

    float label_distance_ {0.5}; // 0 ... 1

    std::set<unsigned int> filter_m3a_values_set_; // dec
    bool filter_m3a_null_wanted_ {false};
    std::set<std::string> filter_ti_values_set_;
    bool filter_ti_null_wanted_ {false};
    std::set<unsigned int> filter_ta_values_set_; // dec
    bool filter_ta_null_wanted_ {false};

    // view point defined labels: rec_num to label level (LOD 1-3)
    std::map<unsigned long, unsigned int> view_point_labels_;

    // per data source settings cached from the config (string + json lookups are too expensive
    // for per row and per frame use), filled on first access, cleared on every change
    struct DSLabelSettings
    {
        std::optional<bool>           wanted;
        std::optional<unsigned int>   line;
        std::optional<LabelDirection> direction;
    };
    std::map<unsigned int, DSLabelSettings> ds_label_settings_; // ds id -> cached settings

    uint64_t config_generation_ {0}; // see configGeneration()

    //std::set<GeometryLeafItemLabels*> item_labels_;
};

}
