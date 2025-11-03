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

#include "viewpresets.h"

#include "view.h"
#include "files.h"
#include "compass.h"
#include "config.h"
#include "timeconv.h"

#include <set>
#include <fstream>

const std::string ViewPresets::TagName        = "name";
const std::string ViewPresets::TagCategory    = "category";
const std::string ViewPresets::TagDescription = "description";
const std::string ViewPresets::TagView        = "view";
const std::string ViewPresets::TagTimestamp   = "timestamp";
const std::string ViewPresets::TagVersion     = "app_version";
const std::string ViewPresets::TagDeployed    = "deployed";
const std::string ViewPresets::TagModified    = "modified";
const std::string ViewPresets::TagConfig      = "view_config";

const std::string ViewPresets::DirPresets     = "view";
const std::string ViewPresets::DirPreviews    = "previews";

const std::string ViewPresets::ExtPreset      = ".json";
const std::string ViewPresets::ExtPreview     = ".png";
const std::string ViewPresets::PrefixPreset   = "preset_";

namespace
{
    std::string viewID(const View* view)
    {
        return view->classId();
    }
}

/**
*/
ViewPresets::ViewPresets() = default;

/**
*/
std::string ViewPresets::uniqueBasename(const Preset& preset) const
{
    traced_assert(!preset.view.empty());
    traced_assert(!preset.name.empty());

    std::string view_preset_dir = viewPresetDir(preset);
    std::string fn_base         = PrefixPreset + Utils::Files::normalizeFilename(preset.name, true);

    //no preset dir yet? => return initial filename
    if (!Utils::Files::directoryExists(view_preset_dir))
        return fn_base;

    auto composePath = [ & ] (const std::string& basename)
    {
        return view_preset_dir + "/" + basename + ExtPreset;
    };

    std::string  fn_base_unique = fn_base;
    unsigned int suffix_cnt     = 1;

    //add numerical suffix until unique filename has been found
    while (Utils::Files::fileExists(composePath(fn_base_unique)))
        fn_base_unique = fn_base + "_" + std::to_string(suffix_cnt++);

    return fn_base_unique;
}

/**
 * Base directory for all view presets.
 */
std::string ViewPresets::viewPresetBaseDir() const
{
    return HOME_PRESET_DIRECTORY + "/" + DirPresets;
}

/**
 * Directory for all presets of a specific view type.
*/
std::string ViewPresets::viewPresetDir(const View* view) const
{
    traced_assert(view);
    return viewPresetBaseDir() + "/" + viewID(view);
}

/**
 * Directory for all presets of a specific view type.
*/
std::string ViewPresets::viewPresetDir(const Preset& preset) const
{
    traced_assert(!preset.view.empty());
    return viewPresetBaseDir() + "/" + preset.view;
}

/**
 * Filename for a view preset.
*/
std::string ViewPresets::viewPresetFilename(const Preset& preset) const
{
    traced_assert(!preset.filename.empty());
    return preset.filename;
}

/**
 * Complete file path for a view preset.
*/
std::string ViewPresets::viewPresetPath(const Preset& preset) const
{
    return viewPresetDir(preset) + "/" + viewPresetFilename(preset);
}

/**
*/
std::string ViewPresets::previewDir(const View* view) const
{
    return viewPresetDir(view);
}

/**
*/
std::string ViewPresets::previewDir(const Preset& preset) const
{
    return viewPresetDir(preset);
}

/**
*/
std::string ViewPresets::previewFilename(const Preset& preset) const
{
    return Utils::Files::replaceExtension(viewPresetFilename(preset), ExtPreview);
}
/**
*/
std::string ViewPresets::previewPath(const Preset& preset) const
{
    return previewDir(preset) + "/" + previewFilename(preset);
}


/**
 * Scans the preset directory for presets and collects them.
 */
bool ViewPresets::scanForPresets()
{
    presets_.clear();

    auto preset_base_dir  = viewPresetBaseDir();

    if (!Utils::Files::directoryExists(preset_base_dir))
    {
        logwrn << "view preset base directory not found";
        return true;
    }

    auto sub_dirs = Utils::Files::getSubdirectories(preset_base_dir);

    for (const auto& sub_dir : sub_dirs)
    {
        std::string view_dir = preset_base_dir + "/" + sub_dir.toStdString();

        auto files = Utils::Files::getFilesInDirectory(view_dir);

        for (const auto& f : files)
        {
            //not a preset file?
            if (!f.startsWith(QString::fromStdString(PrefixPreset)) ||
                !f.endsWith(QString::fromStdString(ExtPreset)))
                continue;

            std::string preset_fn = view_dir + "/" + f.toStdString();
            Preset p;

            bool ok = false;
            try
            {
                ok = readPreset(p, preset_fn);
            }
            catch(...)
            {
                ok = false;
            }
            
            if (!ok)
            {
                logwrn << "Could not read preset '" << f.toStdString() << "'";
                continue;
            }
                
            //view id must match preset view directory
            if (p.view != sub_dir.toStdString())
            {
                logwrn << "Skipping preset '" << f.toStdString() << "' - bad preset configuration";
                continue;
            }

            //key must not exist
            Key key(p.view, p.name);
            traced_assert(presets_.count(key) == 0);

            //store preset
            presets_[ key ] = p;
        }
    }

    loginf << "read " << presets_.size() << " preset(s)";

    return true;
}

/**
 * Reads a preset json file.
*/
bool ViewPresets::readPreset(Preset& p, const std::string& fn)
{
    p = {};

    if (!Utils::Files::fileExists(fn))
        return false;

    //read file to json object
    std::ifstream f(fn);
    if (!f.is_open())
        return false;

    nlohmann::json data = nlohmann::json::parse(f);
    if (data.is_null() || !data.is_object())
        return false;

    //everything there?
    if (!data.contains(TagName)        ||
        !data.contains(TagView)        ||
        !data.contains(TagConfig))
        return false;

    //check if view config is a valid json object
    if (data[ TagConfig ].is_null() || !data[ TagConfig ].is_object())
        return false;

    p.name        = data[ TagName        ];
    p.view        = data[ TagView        ];
    p.view_config = data[ TagConfig      ];

    //view must not be empty
    if (p.view.empty())
        return false;

    p.deployed = false;
    p.modified = false;

    //optional tags
    if (data.contains(TagCategory))
        p.metadata.category = data[ TagCategory ];
    if (data.contains(TagDescription))
        p.metadata.description = data[ TagDescription ];
    if (data.contains(TagTimestamp))
        p.timestamp = data[ TagTimestamp ];
    if (data.contains(TagVersion))
        p.app_version = data[ TagVersion ];
    if (data.contains(TagDeployed))
        p.deployed = data[ TagDeployed ];
    if (data.contains(TagModified))
        p.modified = data[ TagModified ];

    //remember filename the preset has been read from
    p.filename = Utils::Files::getFilenameFromPath(fn);

    //try to read preview image
    std::string preview_path = previewPath(p);
    if (Utils::Files::fileExists(preview_path))
    {
        if (!p.preview.load(QString::fromStdString(preview_path)))
            logwrn << "could not read preview image for preset '" << p.name << "'";
    }

    return true;
}

/**
*/
bool ViewPresets::writePreview(const Preset& preset) const
{
    //nothing to do?
    if (preset.preview.isNull())
        return true;

    //obtain unique preview path
    std::string preview_path = previewPath(preset);

    //write preview
    if (!preset.preview.save(QString::fromStdString(preview_path)))
        return false;

    return true;
}

/**
*/
bool ViewPresets::writePreset(const Preset& preset) const
{
    traced_assert(!preset.filename.empty());
    traced_assert(!preset.view.empty());

    nlohmann::json obj;
    obj[ TagView        ] = preset.view;
    obj[ TagName        ] = preset.name;
    obj[ TagDescription ] = preset.metadata.description;
    obj[ TagCategory    ] = preset.metadata.category;
    obj[ TagDeployed    ] = preset.deployed;
    obj[ TagModified    ] = preset.modified;
    obj[ TagConfig      ] = preset.view_config;
    
    if (!preset.timestamp.empty())
        obj[ TagTimestamp   ] = preset.timestamp;
    if (!preset.app_version.empty())
        obj[ TagVersion ] = preset.app_version;
    
    //obtain unique preset path
    std::string preset_base_dir = viewPresetBaseDir();
    std::string preset_dir      = viewPresetDir(preset);
    std::string preset_path     = viewPresetPath(preset);
    
    //create directories if needed
    if (!Utils::Files::directoryExists(preset_base_dir) &&
        !Utils::Files::createMissingDirectories(preset_base_dir))
        return false;
    
    if (!Utils::Files::directoryExists(preset_dir) &&
        !Utils::Files::createMissingDirectories(preset_dir))
        return false;

    //try to write preview image
    if (!preset.preview.isNull() && !writePreview(preset))
        logwrn << "preview image for view preset '" << preset.name << "' could not be written";

    //write json content
    std::string content = obj.dump(4);
    if (content.empty())
        return false;

    //@TODO: before writing make a backup copy of the file we could revert to if the write fails
    std::ofstream fileout(preset_path, std::ios::out);
    if (!fileout.is_open())
        return false;

    fileout << content;
    if (!fileout)
        return false;

    return true;
}

/**
 * Creates a new preset for the given view and optionally a preview image.
 */
bool ViewPresets::createPreset(const View* view, 
                               const std::string& name,
                               const PresetMetadata& metadata,
                               bool create_preview)
{
    traced_assert(view);
    traced_assert(!name.empty());

    //config preset
    Preset p;
    p.name     = name;
    p.view     = viewID(view);
    p.metadata = metadata;

    //update view config + preview
    updatePresetConfig(p, view, create_preview);

    //create configured preset
    return createPreset(p, nullptr);
}

/**
 * Creates the given preset on disk and adds it to the collection of presets.
 * The view can be passed to set the preset's view id (otherwise it needs to be set beforehand).
 */
bool ViewPresets::createPreset(const Preset& preset, const View* view)
{
    return createPreset(preset, view, true);
}

/**
 * Creates the given preset on disk and adds it to the collection of presets.
 * The view can be passed to set the preset's view id (otherwise it needs to be set beforehand).
 * Internal version.
 */
bool ViewPresets::createPreset(const Preset& preset, const View* view, bool signal_changes)
{
    traced_assert(!preset.name.empty());

    Preset p = preset;

    //presets created in compass are never deployed presets (and thus modified also makes no sense)
    p.deployed = false;
    p.modified = false;

    //update view info?
    if (view)
        p.view = viewID(view);

    traced_assert(!p.view.empty());

    auto key = p.key();
    traced_assert(presets_.count(key) == 0);

    //create unique filename
    p.filename = uniqueBasename(p) + ExtPreset;

    //try to write
    if (!writePreset(p))
    {
        logwrn << "could not write view preset '" << p.name << "'";
        return false;
    }

    //add to presets only if write succeeds
    presets_[ key ] = p;

    if (signal_changes)
    {
        emit presetAdded(key);
        emit presetEdited(EditAction(EditMode::Add, key, {}, true, view));
    }

    return true;
}


/**
 * Removes the preset of the given key.
 */
void ViewPresets::removePreset(const Key& key, const View* view)
{
    removePreset(key, view, true);
}

/**
 * Removes the preset of the given key.
 * Internal version.
 */
void ViewPresets::removePreset(const Key& key, const View* view, bool signal_changes)
{
    traced_assert(presets_.count(key) > 0);

    const auto& preset = presets_.at(key);

    std::string preset_path  = viewPresetPath(preset);
    std::string preview_path = previewPath(preset);

    traced_assert(Utils::Files::fileExists(preset_path));

    //delete preview if exists
    if (Utils::Files::fileExists(preview_path))
        Utils::Files::deleteFile(preview_path);

    //delete preset
    Utils::Files::deleteFile(preset_path);

    //remove preset
    presets_.erase(key);

    if (signal_changes)
    {
        emit presetRemoved(key);
        emit presetEdited(EditAction(EditMode::Remove, key, {}, false, view));
    }
}

/**
 * Renames the preset of the given key to the given name.
 * Will cause the original file to be deleted and a file to be generated under the new name.
 */
bool ViewPresets::renamePreset(const Key& key, const std::string& new_name, const View* view)
{
    return renamePreset(key, new_name, view, true);
}

/**
 * Renames the preset of the given key to the given name.
 * Will cause the original file to be deleted and a file to be generated under the new name.
 * Internal version.
 */
bool ViewPresets::renamePreset(const Key& key, const std::string& new_name, const View* view, bool signal_changes)
{
    traced_assert(presets_.count(key) > 0);
    traced_assert(!new_name.empty() && new_name != key.second);

    //copy preset and set new name
    Preset preset = presets_.at(key);
    preset.name    = new_name;

    //create preset under new name
    if (!createPreset(preset, nullptr, false))
        return false;

    //only if creation succeeds remove preset under old name
    removePreset(key, view, false);

    if (signal_changes)
    {
        emit presetRenamed(key, preset.key());
        emit presetEdited(EditAction(EditMode::Rename, key, preset.key(), false, view));
    }

    return true;
}

/**
*/
bool ViewPresets::copyPreset(const Key& key,
                             const std::string& new_name,
                             const boost::optional<PresetMetadata>& new_metadata,
                             const View* view)
{
    return copyPreset(key, new_name, new_metadata, view, true);
}

/**
*/
bool ViewPresets::copyPreset(const Key& key,
                             const std::string& new_name,
                             const boost::optional<PresetMetadata>& new_metadata,
                             const View* view, 
                             bool signal_changes)
{
    traced_assert(presets_.count(key) > 0);
    traced_assert(!new_name.empty() && new_name != key.second);

    //copy preset and change name
    Preset preset = presets_.at(key);
    preset.name = new_name;

    //if new metadata is provided store it
    if (new_metadata.has_value())
        preset.metadata = new_metadata.value();

    //create preset under new name
    if (!createPreset(preset, nullptr, false))
        return false;

    if (signal_changes)
    {
        emit presetCopied(key, preset.key());
        emit presetEdited(EditAction(EditMode::Copy, key, preset.key(), false, view));
    }

    return true;
}

/**
 * Updates the preset under the given key to the passed preset.
 * Might cause a rename. If the update fails the old preset version will be restored.
 */
bool ViewPresets::updatePreset(const Key& key, 
                               UpdateMode mode,
                               const Preset* preset,
                               const View* view,   
                               bool update_config_from_view,
                               bool update_preview)
{
    return updatePreset(key, preset, view, update_config_from_view, mode, update_preview, true);
}

/**
 * Updates the preset under the given key to the data of the passed preset.
 * If a view is given the presets config will be updated to the view config.
 * Internal version.
 */
bool ViewPresets::updatePreset(const Key& key, 
                               const Preset* preset,
                               const View* view, 
                               bool update_config_from_view, 
                               UpdateMode mode,
                               bool update_preview, 
                               bool signal_changes)
{
    traced_assert(presets_.count(key) > 0);

    auto& preset_cur = presets_.at(key);

    //backup current preset version
    auto preset_backup = preset_cur;

    //copy description etc from preset
    if (mode == UpdateMode::MetaData || mode == UpdateMode::All)
    {
        if (preset)
        {
            preset_cur.metadata = preset->metadata;
        }
        else
        {
            //no source provided for metadata update
            return false;
        }
    }

    bool config_changed = false;

    //change config?
    if (mode == UpdateMode::ViewConfig || mode == UpdateMode::All)
    {
        if (update_config_from_view)
        {
            traced_assert(view);
            traced_assert(viewID(view) == preset_cur.view);

            //view provided => update to view config
            config_changed = updatePresetConfig(preset_cur, view, update_preview);
        }
        else if (preset && !preset->view_config.is_null())
        {
            traced_assert(preset_cur.view == preset->view);

            config_changed = preset_cur.view_config != preset->view_config;

            //config provided in passed preset => copy config related data
            preset_cur.view_config = preset->view_config;
            preset_cur.preview     = preset->preview;
            preset_cur.timestamp   = preset->timestamp;
            preset_cur.app_version = preset->app_version;
        }
        else
        {
            //no source provided for config update
            return false;
        }
    }

    //if deployed now set to modified
    if (preset_cur.deployed)
        preset_cur.modified = true;

    //try to write
    if (!writePreset(preset_cur))
    {
        //fail => revert to old version
        preset_cur = preset_backup;
        return false;
    }

    if (signal_changes)
    {
        emit presetUpdated(key);
        emit presetEdited(EditAction(EditMode::Update, key, {}, config_changed, view));
    }

    return true;
}

/**
 * Writes the preset of the given key to disk.
 * Internal version.
 */
bool ViewPresets::writePreset(const Key& key) const
{
    traced_assert(presets_.count(key) != 0);

    const Preset& p = presets_.at(key);

    if (!writePreset(p))
    {
        logwrn << "could not write view preset '" << key.second << "'";
        return false;
    }

    return true;
}

/**
 * Updates the presets "stamp", meaning timestamp and app version.
 * This stamp is usually updated when setting a new view config.
*/
void ViewPresets::updatePresetStamp(Preset& preset)
{
    //add timestamp
    preset.timestamp = Utils::Time::toString(Utils::Time::currentUTCTime());

    //add app version
    preset.app_version = COMPASS::instance().config().getString("version");
}

/**
 * Generates a preview image of the given view.
 */
QImage ViewPresets::renderPreview(const View* view)
{
    traced_assert(view);
    
    auto preview = view->renderData();
    return preview.scaled(PreviewMaxSize, PreviewMaxSize, Qt::AspectRatioMode::KeepAspectRatio);
}

/**
 * Updates the presets config to the passed view's config, and optionally creates a preview image.
 * Preferably use this method to update the presets view config, since it also updates the preset's stamp
 * on config change.
 */
bool ViewPresets::updatePresetConfig(Preset& preset, const View* view, bool update_preview)
{
    traced_assert(view);
    traced_assert(preset.view.empty() || viewID(view) == preset.view);

    //collect json config
    nlohmann::json new_cfg;
    view->generateJSON(new_cfg, Configurable::JSONExportType::Preset);

    bool cfg_changed = new_cfg != preset.view_config;

    preset.view_config = new_cfg;

    if (update_preview)
        preset.preview = renderPreview(view);

    //update signature on config modify
    updatePresetStamp(preset);

    return cfg_changed;
}

/**
*/
bool ViewPresets::keyValid(const Key& key)
{
    return (!key.first.empty() && !key.second.empty());
}

/**
*/
bool ViewPresets::keyIsView(const Key& key, const View* view)
{
    traced_assert(view);
    return (viewID(view) == key.first);
}

/**
*/
const ViewPresets::Presets& ViewPresets::presets() const
{
    return presets_;
}

/**
*/
ViewPresets::Presets& ViewPresets::presets()
{
    return presets_;
}

/**
 * Returns the preset keys for presets belonging to the given view.
 */
std::vector<ViewPresets::Key> ViewPresets::keysFor(const View* view) const
{
    traced_assert(view);

    std::vector<Key> keys;

    for (const auto& p : presets_)
        if (p.second.view == viewID(view))
            keys.push_back(p.first);

    return keys;
}

/**
 * Returns the preset keys for presets belonging to the given category.
 */
std::vector<ViewPresets::Key> ViewPresets::keysFor(const std::string& category) const
{
    std::vector<Key> keys;

    for (const auto& p : presets_)
        if (p.second.metadata.category == category)
            keys.push_back(p.first);

    return keys;
}

/**
 * Returns the preset keys for presets belonging to the given view and category.
 */
std::vector<ViewPresets::Key> ViewPresets::keysFor(const View* view, 
                                                   const std::string& category) const
{
    traced_assert(view);

    std::vector<Key> keys;

    for (const auto& p : presets_)
        if (p.second.view == viewID(view) && p.second.metadata.category == category)
            keys.push_back(p.first);

    return keys;
}

/**
 * Returns all existing categories.
 */
std::vector<std::string> ViewPresets::categories() const
{
    std::set<std::string> categories;

    for (const auto& p : presets_)
        categories.insert(p.second.metadata.category);

    return std::vector<std::string>(categories.begin(), categories.end());
}

/**
 * Returns all categories existing for the given view.
 */
std::vector<std::string> ViewPresets::categories(const View* view) const
{
    traced_assert(view);

    std::set<std::string> categories;

    for (const auto& p : presets_)
        if (p.second.view == viewID(view))
            categories.insert(p.second.metadata.category);

    return std::vector<std::string>(categories.begin(), categories.end());
}

/**
*/
bool ViewPresets::hasPreset(const View* view, const std::string& name) const
{
    traced_assert(view);

    return hasPreset(Key(viewID(view), name));
}

/**
*/
bool ViewPresets::hasPreset(const Key& key) const
{
    return presets_.count(key) > 0;
}
