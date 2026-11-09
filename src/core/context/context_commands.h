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

#include "rtcommand.h"
#include "rtcommand_macros.h"

namespace context
{
    class DBContextManager;
}

namespace context_cmd
{

extern void init_context_commands(rtcommand::RTCommandRegistry& registry, context::DBContextManager& ctx_man);

// create_context <name>
struct RTCommandCreateContext : public rtcommand::RTCommand
{
    explicit RTCommandCreateContext(context::DBContextManager& ctx_man);

    std::string name_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(create_context, "creates a new data context with the given name")
    DECLARE_RTCOMMAND_OPTIONS
};

// set_context <name>
struct RTCommandSetContext : public rtcommand::RTCommand
{
    explicit RTCommandSetContext(context::DBContextManager& ctx_man);

    std::string name_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(set_context, "sets the active data context")
    DECLARE_RTCOMMAND_OPTIONS
};

// delete_context <name>
struct RTCommandDeleteContext : public rtcommand::RTCommand
{
    explicit RTCommandDeleteContext(context::DBContextManager& ctx_man);

    std::string name_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(delete_context, "deletes the data context with the given name")
    DECLARE_RTCOMMAND_OPTIONS
};

// list_contexts
struct RTCommandListContexts : public rtcommand::RTCommand
{
    explicit RTCommandListContexts(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(list_contexts, "lists all data context names")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// get_context_info
struct RTCommandGetContextInfo : public rtcommand::RTCommand
{
    explicit RTCommandGetContextInfo(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(get_context_info, "returns info about the active data context")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// get_context <name>
struct RTCommandGetContext : public rtcommand::RTCommand
{
    explicit RTCommandGetContext(context::DBContextManager& ctx_man);

    std::string name_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(get_context, "returns the full data context with the given name as JSON")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_ffts_json <file>
struct RTCommandImportFFTsJSON : public rtcommand::RTCommand
{
    explicit RTCommandImportFFTsJSON(context::DBContextManager& ctx_man);

    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(import_ffts_json, "imports FFTs from a JSON file")
    DECLARE_RTCOMMAND_OPTIONS
};

// import_context <zip file>
struct RTCommandImportContext : public rtcommand::RTCommand
{
    explicit RTCommandImportContext(context::DBContextManager& ctx_man);

    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(import_context, "imports a data context from a zip file")
    DECLARE_RTCOMMAND_OPTIONS
};

// delete_all_sectors
struct RTCommandDeleteAllSectors : public rtcommand::RTCommand
{
    explicit RTCommandDeleteAllSectors(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(delete_all_sectors, "deletes all sectors from the active context")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// delete_all_ffts
struct RTCommandDeleteAllFFTs : public rtcommand::RTCommand
{
    explicit RTCommandDeleteAllFFTs(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(delete_all_ffts, "deletes all FFTs from the active context")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// import_sectors_gdal <file> [--layer=name] [--color=#hex] [--exclude]
struct RTCommandImportSectorsGDAL : public rtcommand::RTCommand
{
    explicit RTCommandImportSectorsGDAL(context::DBContextManager& ctx_man);

    std::string filename_;
    std::string layer_name_;
    std::string color_str_;
    bool exclude_{false};

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(import_sectors_gdal, "imports sectors from a GDAL-supported file (SHP, GeoJSON, KML)")
    DECLARE_RTCOMMAND_OPTIONS
};

// export_sectors_json <file>
struct RTCommandExportSectorsJSON : public rtcommand::RTCommand
{
    explicit RTCommandExportSectorsJSON(context::DBContextManager& ctx_man);

    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(export_sectors_json, "exports sectors to a JSON file")
    DECLARE_RTCOMMAND_OPTIONS
};

// export_data_sources_json <file>
struct RTCommandExportDataSourcesJSON : public rtcommand::RTCommand
{
    explicit RTCommandExportDataSourcesJSON(context::DBContextManager& ctx_man);

    std::string filename_;

    virtual rtcommand::IsValid valid() const override;

protected:
    virtual bool run_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(export_data_sources_json, "exports data sources to a JSON file")
    DECLARE_RTCOMMAND_OPTIONS
};

} // namespace context_cmd
