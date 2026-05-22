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

#include "json.hpp"

#include <map>
#include <string>

#include <boost/optional.hpp>

#include <QImage>

class View;

/**
 * Collection of presets for all kinds of views.
 */
class ViewPresets : public QObject
{
    Q_OBJECT
public:
    typedef std::pair<std::string, std::string> Key;

    /**
     * What data to update in a preset.
     */
    enum class UpdateMode
    {
        MetaData = 0, // update metadata: name, category, etc.
        ViewConfig,   // update view configuration
        All           // update everything
    };

    /**
     * Type of preset edit action.
     */
    enum class EditMode
    {
        Add,    // new preset added
        Remove, // preset removed
        Update, // preset data updated
        Copy,   // preset copied to a new preset
        Rename  // preset renamed
    };

    /**
     * Preset edit action.
     */
    struct EditAction
    {
        EditAction(EditMode m, Key k, Key k2, bool cfg_change, const View* v) 
        :   mode          (m)
        ,   key           (k)
        ,   key2          (k2)
        ,   config_changed(cfg_change)
        ,   view          (v) {}

        bool valid() const
        {
            //at least the key should be valid
            return ViewPresets::keyValid(key);
        }

        EditMode    mode;           // type of edit action
        Key         key;            // key of edited preset
        Key         key2;           // new key of edited preset (e.g. rename, copy)
        bool        config_changed; // view config has changed during the edit action
        const View* view;           // view that issued the edit action
    };

    /**
     * Metadata attached to a preset.
     */
    struct PresetMetadata
    {
        std::string category;    // category of the preset
        std::string description; // description of what the preset is about
    };

    /**
     * Struct defining a view preset.
     */
    struct Preset
    {
        //returns a unique key, being a combination of the view's class id and the preset name
        Key key() const { return Key(view, name); }

        bool isDeployed() const { return deployed; }
        bool isModified() const { return deployed && modified; }
        bool isUnmodified() const { return !deployed || !modified; }

        //serialized
        std::string    name;             // unique name of the preset
        PresetMetadata metadata;         // preset metadata info
        std::string    view;             // view the preset belongs to (= the view's configurable class id)
        std::string    timestamp;        // timestamp the preset config was generated
        std::string    app_version;      // version of the application the preset has been generated with
        bool           deployed = false; // preset was deployed with compass
        bool           modified = false; // preset has been modified from its original version (for deployed presets)
        nlohmann::json view_config;      // view configuration as json blob

        //not serialized
        std::string    filename;         // filename of the preset on disk
        QImage         preview;          // a preview image generated from the view using the stored configuration
    };

    typedef std::map<Key, Preset> Presets;

    ViewPresets();
    virtual ~ViewPresets() = default;

    bool scanForPresets();

    bool createPreset(View* view,
                      const std::string& name,
                      const PresetMetadata& metadata,
                      bool create_preview = true);
    bool createPreset(const Preset& preset, 
                      const View* view = nullptr);
    void removePreset(const Key& key, 
                      const View* view = nullptr);
    bool renamePreset(const Key& key, 
                      const std::string& new_name,
                      const View* view = nullptr);
    bool copyPreset(const Key& key,
                    const std::string& new_name,
                    const boost::optional<PresetMetadata>& new_metadata,
                    const View* view = nullptr);
    bool updatePreset(const Key& key,
                      UpdateMode mode,
                      const Preset* preset,
                      View* view,
                      bool update_config_from_view = true,
                      bool update_preview = true);
    
    bool hasPreset(const View* view, const std::string& name) const;
    bool hasPreset(const Key& key) const;
    bool nameExists(const std::string& name, const View* view) const;

    static void updatePresetStamp(Preset& preset, const std::string& app_version);
    static bool updatePresetConfig(Preset& preset, View* view, bool update_preview = true);
    static QImage renderPreview(const View* view);

    static bool keyValid(const Key& key);
    static bool keyIsView(const Key& key, const View* view);

    const Presets& presets() const;
    Presets& presets();

    std::vector<Key> keysFor(const View* view) const;
    std::vector<Key> keysFor(const std::string& category) const;
    std::vector<Key> keysFor(const View* view, 
                             const std::string& category) const;

    std::vector<std::string> categories() const;
    std::vector<std::string> categories(const View* view) const;

    static const std::string TagName;
    static const std::string TagCategory;
    static const std::string TagDescription;
    static const std::string TagView;
    static const std::string TagTimestamp;
    static const std::string TagVersion;
    static const std::string TagDeployed;
    static const std::string TagConfig;
    static const std::string TagModified;

    static const std::string DirPresets;
    static const std::string DirPreviews;

    static const std::string ExtPreset;
    static const std::string ExtPreview;
    static const std::string PrefixPreset;

    static const int PreviewMaxSize = 100;

signals:
    void presetUpdated(Key key);
    void presetAdded(Key key);
    void presetRemoved(Key key);
    void presetCopied(Key key_old, Key key_new);
    void presetRenamed(Key key_old, Key key_new);

    void presetEdited(EditAction ea);

private:
    bool writePreview(const Preset& preset) const;
    bool writePreset(const Preset& preset) const;
    bool writePreset(const Key& key) const;
    bool readPreset(Preset& p, const std::string& fn);

    //internal versions
    bool createPreset(const Preset& preset, 
                      const View* view, 
                      bool signal_changes);
    void removePreset(const Key& key, 
                      const View* view, 
                      bool signal_changes);
    bool renamePreset(const Key& key, 
                      const std::string& new_name, 
                      const View* view, 
                      bool signal_changes);
    bool copyPreset(const Key& key,
                    const std::string& new_name,
                    const boost::optional<PresetMetadata>& new_metadata,
                    const View* view, 
                    bool signal_changes);
    bool updatePreset(const Key& key,
                      const Preset* preset,
                      View* view,
                      bool update_config_from_view,
                      UpdateMode mode,
                      bool update_preview,
                      bool signal_changes);

    std::string uniqueBasename(const Preset& preset) const;

    std::string viewPresetBaseDir() const;

    std::string viewPresetDir(const View* view) const;
    std::string viewPresetDir(const Preset& preset) const;
    std::string viewPresetFilename(const Preset& preset) const;
    std::string viewPresetPath(const Preset& preset) const;

    std::string previewDir(const View* view) const;
    std::string previewDir(const Preset& preset) const;
    std::string previewFilename(const Preset& preset) const;
    std::string previewPath(const Preset& preset) const;
    
    Presets presets_;
};
