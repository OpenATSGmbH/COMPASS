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

#include <QVariant>

#include <memory>
#include <string>
#include <vector>

namespace context
{

/**
 * Base class for tree items in the DBContext edit dialog tree.
 */
class DBContextEditTreeItem
{
public:
    explicit DBContextEditTreeItem(DBContextEditTreeItem* parent = nullptr)
        : parent_(parent) {}
    virtual ~DBContextEditTreeItem() = default;

    virtual DBContextEditTreeItem* child(int row) { return row < childCount() ? children_.at(row).get() : nullptr; }
    virtual int childCount() const { return static_cast<int>(children_.size()); }
    int columnCount() const { return 1; }
    virtual QVariant data(int /*column*/) const = 0;
    DBContextEditTreeItem* parentItem() { return parent_; }

    int row() const
    {
        if (!parent_)
            return 0;
        for (int i = 0; i < static_cast<int>(parent_->children_.size()); ++i)
        {
            if (parent_->children_[i].get() == this)
                return i;
        }
        return 0;
    }

    void appendChild(std::unique_ptr<DBContextEditTreeItem> child)
    {
        children_.push_back(std::move(child));
    }

protected:
    DBContextEditTreeItem* parent_{nullptr};
    std::vector<std::unique_ptr<DBContextEditTreeItem>> children_;
};

/**
 * Invisible root item — holds the top-level groups.
 */
class RootItem : public DBContextEditTreeItem
{
public:
    RootItem() : DBContextEditTreeItem(nullptr) {}
    QVariant data(int /*column*/) const override { return {}; }
};

/**
 * Top-level group header (Data Sources, Sector Layers, FFTs).
 */
class GroupItem : public DBContextEditTreeItem
{
public:
    enum GroupType { DataSources, SectorLayers, FFTs };

    GroupItem(GroupType type, DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent), type_(type) {}

    QVariant data(int /*column*/) const override
    {
        switch (type_)
        {
        case DataSources:  return "Data Sources";
        case SectorLayers: return "Sector Layers";
        case FFTs:         return "FFTs";
        }
        return {};
    }

    GroupType groupType() const { return type_; }

private:
    GroupType type_;
};

/**
 * Single leaf node for ASTERIX Configuration.
 */
class ASTERIXConfigLeafItem : public DBContextEditTreeItem
{
public:
    explicit ASTERIXConfigLeafItem(DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent) {}

    int childCount() const override { return 0; }
    QVariant data(int /*column*/) const override { return "ASTERIX Configuration"; }
};

/**
 * Data source type subgroup (e.g. "Radar", "ADSB", "MLAT").
 */
class DSTypeGroupItem : public DBContextEditTreeItem
{
public:
    DSTypeGroupItem(const std::string& ds_type, DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent), ds_type_(ds_type) {}

    QVariant data(int /*column*/) const override
    {
        return QString::fromStdString(ds_type_);
    }

    const std::string& dsType() const { return ds_type_; }

private:
    std::string ds_type_;
};

/**
 * A single data source entry.
 */
class DataSourceItem : public DBContextEditTreeItem
{
public:
    DataSourceItem(unsigned int ds_id, const std::string& name,
                   unsigned int sac, unsigned int sic,
                   DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent)
        , ds_id_(ds_id), name_(name), sac_(sac), sic_(sic) {}

    int childCount() const override { return 0; }

    QVariant data(int /*column*/) const override
    {
        return QString::fromStdString(name_) + " (" +
               QString::number(sac_) + "/" + QString::number(sic_) + ")";
    }

    unsigned int dsId() const { return ds_id_; }

private:
    unsigned int ds_id_;
    std::string name_;
    unsigned int sac_;
    unsigned int sic_;
};

/**
 * A sector layer subgroup (contains SectorItems).
 */
class SectorLayerItem : public DBContextEditTreeItem
{
public:
    SectorLayerItem(const std::string& layer_name, DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent), layer_name_(layer_name) {}

    QVariant data(int /*column*/) const override
    {
        return QString::fromStdString(layer_name_);
    }

    const std::string& layerName() const { return layer_name_; }

private:
    std::string layer_name_;
};

/**
 * A single sector entry.
 */
class SectorItem : public DBContextEditTreeItem
{
public:
    SectorItem(unsigned int sector_id, const std::string& name, DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent), sector_id_(sector_id), name_(name) {}

    int childCount() const override { return 0; }

    QVariant data(int /*column*/) const override
    {
        return QString::fromStdString(name_);
    }

    unsigned int sectorId() const { return sector_id_; }

private:
    unsigned int sector_id_;
    std::string name_;
};

/**
 * A single FFT entry.
 */
class FFTItem : public DBContextEditTreeItem
{
public:
    FFTItem(const std::string& name, DBContextEditTreeItem* parent)
        : DBContextEditTreeItem(parent), name_(name) {}

    int childCount() const override { return 0; }

    QVariant data(int /*column*/) const override
    {
        return QString::fromStdString(name_);
    }

    const std::string& fftName() const { return name_; }

private:
    std::string name_;
};

} // namespace context
