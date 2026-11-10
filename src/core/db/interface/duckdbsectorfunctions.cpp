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

#include "duckdbsectorfunctions.h"
#include "sector.h"
#include "sectorinsidecache.h"
#include "logger.h"

#include <cstring>

namespace
{
    /**
     * Bound data of the cache function, owned by the DuckDB function object.
     */
    struct CacheFunctionInfo
    {
        std::shared_ptr<SectorInsideCache> cache;
    };

    void destroyCacheFunctionInfo(void* data)
    {
        delete static_cast<CacheFunctionInfo*>(data);
    }

    /**
     * Marks a result row null.
     */
    void setResultNull(duckdb_vector output, uint64_t*& result_validity, idx_t row)
    {
        if (!result_validity)
        {
            duckdb_vector_ensure_validity_writable(output);
            result_validity = duckdb_vector_get_validity(output);
        }

        duckdb_validity_set_row_invalid(result_validity, row);
    }

    /**
     * Evaluates one input chunk of the cache function. The input vectors are layer name as
     * VARCHAR, latitude, longitude, barometric altitude and the delta as DOUBLE, the output is
     * BOOLEAN. A null altitude means no altitude, the altitude limits are then not checked.
     * The layer and the delta are constants in practice, so the prepared layer is looked up
     * only when they change from one row to the next.
     */
    void cacheFunction(duckdb_function_info info, duckdb_data_chunk input, duckdb_vector output)
    {
        const auto* function_info =
            static_cast<const CacheFunctionInfo*>(duckdb_scalar_function_get_extra_info(info));

        const idx_t num_rows = duckdb_data_chunk_get_size(input);

        duckdb_vector layer_vector = duckdb_data_chunk_get_vector(input, 0);
        duckdb_vector lat_vector   = duckdb_data_chunk_get_vector(input, 1);
        duckdb_vector lon_vector   = duckdb_data_chunk_get_vector(input, 2);
        duckdb_vector alt_vector   = duckdb_data_chunk_get_vector(input, 3);
        duckdb_vector delta_vector = duckdb_data_chunk_get_vector(input, 4);

        duckdb_string_t* layers     = static_cast<duckdb_string_t*>(duckdb_vector_get_data(layer_vector));
        const double*    latitudes  = static_cast<const double*>(duckdb_vector_get_data(lat_vector));
        const double*    longitudes = static_cast<const double*>(duckdb_vector_get_data(lon_vector));
        const double*    altitudes  = static_cast<const double*>(duckdb_vector_get_data(alt_vector));
        const double*    deltas     = static_cast<const double*>(duckdb_vector_get_data(delta_vector));

        //null if no row is null
        uint64_t* layer_validity = duckdb_vector_get_validity(layer_vector);
        uint64_t* lat_validity   = duckdb_vector_get_validity(lat_vector);
        uint64_t* lon_validity   = duckdb_vector_get_validity(lon_vector);
        uint64_t* alt_validity   = duckdb_vector_get_validity(alt_vector);
        uint64_t* delta_validity = duckdb_vector_get_validity(delta_vector);

        bool*     result          = static_cast<bool*>(duckdb_vector_get_data(output));
        uint64_t* result_validity = nullptr;

        //the prepared layer of the previous row
        std::shared_ptr<const SectorInsideCache::PreparedLayer> prepared;
        std::string prepared_name;
        double      prepared_delta = 0.0;

        for (idx_t row = 0; row < num_rows; ++row)
        {
            if ((layer_validity && !duckdb_validity_row_is_valid(layer_validity, row)) ||
                (lat_validity   && !duckdb_validity_row_is_valid(lat_validity,   row)) ||
                (lon_validity   && !duckdb_validity_row_is_valid(lon_validity,   row)) ||
                (delta_validity && !duckdb_validity_row_is_valid(delta_validity, row)))
            {
                setResultNull(output, result_validity, row);
                result[row] = false;
                continue;
            }

            const char*    layer_data = duckdb_string_t_data(&layers[row]);
            const uint32_t layer_len  = duckdb_string_t_length(layers[row]);
            const double   delta      = deltas[row];

            if (!prepared || delta != prepared_delta || prepared_name.size() != layer_len ||
                std::memcmp(prepared_name.data(), layer_data, layer_len) != 0)
            {
                prepared_name.assign(layer_data, layer_len);
                prepared_delta = delta;
                prepared       = function_info->cache->prepared(prepared_name, prepared_delta);

                if (!prepared)
                {
                    std::string msg = "sector_inside: unknown sector layer '" + prepared_name + "'";
                    duckdb_scalar_function_set_error(info, msg.c_str());
                    return;
                }
            }

            const bool has_altitude = !alt_validity || duckdb_validity_row_is_valid(alt_validity, row);

            result[row] = prepared->isInside(latitudes[row], longitudes[row],
                                             has_altitude, has_altitude ? altitudes[row] : 0.0);
        }
    }
}

/**
 */
bool DuckDBSectorFunctions::registerCacheFunction(duckdb_connection connection,
                                                  const std::shared_ptr<SectorInsideCache>& cache,
                                                  std::string* error)
{
    auto fail = [&](const std::string& msg)
    {
        logerr << msg;

        if (error)
            *error = msg;

        return false;
    };

    if (!connection)
        return fail("sector inside cache function: no connection");

    if (!cache)
        return fail("sector inside cache function: no cache");

    const std::string function_name = SectorInsideCache::DBFunctionName;

    duckdb_scalar_function function = duckdb_create_scalar_function();
    duckdb_scalar_function_set_name(function, function_name.c_str());

    duckdb_logical_type varchar_type = duckdb_create_logical_type(DUCKDB_TYPE_VARCHAR);
    duckdb_logical_type double_type  = duckdb_create_logical_type(DUCKDB_TYPE_DOUBLE);
    duckdb_logical_type bool_type    = duckdb_create_logical_type(DUCKDB_TYPE_BOOLEAN);

    duckdb_scalar_function_add_parameter(function, varchar_type);
    duckdb_scalar_function_add_parameter(function, double_type); //latitude
    duckdb_scalar_function_add_parameter(function, double_type); //longitude
    duckdb_scalar_function_add_parameter(function, double_type); //barometric altitude, may be null
    duckdb_scalar_function_add_parameter(function, double_type); //delta
    duckdb_scalar_function_set_return_type(function, bool_type);

    //a null altitude is a valid input, the function handles nulls itself
    duckdb_scalar_function_set_special_handling(function);

    duckdb_destroy_logical_type(&varchar_type);
    duckdb_destroy_logical_type(&double_type);
    duckdb_destroy_logical_type(&bool_type);

    //owned by the function object from here on, freed with it
    auto* function_info = new CacheFunctionInfo{cache};

    duckdb_scalar_function_set_extra_info(function, function_info, destroyCacheFunctionInfo);
    duckdb_scalar_function_set_function(function, cacheFunction);

    duckdb_state state = duckdb_register_scalar_function(connection, function);
    duckdb_destroy_scalar_function(&function);

    if (state != DuckDBSuccess)
        return fail("sector inside cache function '" + function_name + "': registration failed");

    loginf << "registered sector inside cache function '" << function_name << "'";

    return true;
}

namespace
{
    /**
     * Bound data of one registered function, owned by the DuckDB function object.
     */
    struct InsideFunctionInfo
    {
        std::vector<std::shared_ptr<Sector>> sectors;
        double                               delta_deg = 0.0;
    };

    void destroyInsideFunctionInfo(void* data)
    {
        delete static_cast<InsideFunctionInfo*>(data);
    }

    /**
     * Evaluates one input chunk. The input vectors are latitude and longitude as DOUBLE,
     * the output vector is BOOLEAN.
     */
    void insideFunction(duckdb_function_info info, duckdb_data_chunk input, duckdb_vector output)
    {
        const auto* function_info =
            static_cast<const InsideFunctionInfo*>(duckdb_scalar_function_get_extra_info(info));

        const idx_t num_rows = duckdb_data_chunk_get_size(input);

        duckdb_vector lat_vector = duckdb_data_chunk_get_vector(input, 0);
        duckdb_vector lon_vector = duckdb_data_chunk_get_vector(input, 1);

        const double* latitudes  = static_cast<const double*>(duckdb_vector_get_data(lat_vector));
        const double* longitudes = static_cast<const double*>(duckdb_vector_get_data(lon_vector));

        //null if no row is null
        uint64_t* lat_validity = duckdb_vector_get_validity(lat_vector);
        uint64_t* lon_validity = duckdb_vector_get_validity(lon_vector);

        bool*     result          = static_cast<bool*>(duckdb_vector_get_data(output));
        uint64_t* result_validity = nullptr;

        for (idx_t row = 0; row < num_rows; ++row)
        {
            if ((lat_validity && !duckdb_validity_row_is_valid(lat_validity, row)) ||
                (lon_validity && !duckdb_validity_row_is_valid(lon_validity, row)))
            {
                if (!result_validity)
                {
                    duckdb_vector_ensure_validity_writable(output);
                    result_validity = duckdb_vector_get_validity(output);
                }

                duckdb_validity_set_row_invalid(result_validity, row);
                result[row] = false;
                continue;
            }

            bool inside = false;

            for (const auto& sector : function_info->sectors)
            {
                if (sector->isExclusionSector())
                    continue;

                if (sector->isInside(latitudes[row], longitudes[row], function_info->delta_deg))
                {
                    inside = true;
                    break;
                }
            }

            result[row] = inside;
        }
    }
}

/**
 */
bool DuckDBSectorFunctions::registerInsideFunction(duckdb_connection connection,
                                                   const std::string& name,
                                                   const std::vector<std::shared_ptr<Sector>>& sectors,
                                                   double delta_deg,
                                                   std::string* error)
{
    auto fail = [&](const std::string& msg)
    {
        logerr << msg;

        if (error)
            *error = msg;

        return false;
    };

    if (!connection)
        return fail("sector inside function '" + name + "': no connection");

    if (name.empty())
        return fail("sector inside function: empty name");

    if (sectors.empty())
        return fail("sector inside function '" + name + "': no sectors");

    //prepare the fast tests before the function can be called from several threads
    for (const auto& sector : sectors)
    {
        if (!sector)
            return fail("sector inside function '" + name + "': null sector");

        sector->createFastInsideTest(delta_deg);
    }

    duckdb_scalar_function function = duckdb_create_scalar_function();
    duckdb_scalar_function_set_name(function, name.c_str());

    duckdb_logical_type double_type = duckdb_create_logical_type(DUCKDB_TYPE_DOUBLE);
    duckdb_logical_type bool_type   = duckdb_create_logical_type(DUCKDB_TYPE_BOOLEAN);

    duckdb_scalar_function_add_parameter(function, double_type);
    duckdb_scalar_function_add_parameter(function, double_type);
    duckdb_scalar_function_set_return_type(function, bool_type);

    duckdb_destroy_logical_type(&double_type);
    duckdb_destroy_logical_type(&bool_type);

    //owned by the function object from here on, freed with it
    auto* function_info = new InsideFunctionInfo{sectors, delta_deg};

    duckdb_scalar_function_set_extra_info(function, function_info, destroyInsideFunctionInfo);
    duckdb_scalar_function_set_function(function, insideFunction);

    duckdb_state state = duckdb_register_scalar_function(connection, function);
    duckdb_destroy_scalar_function(&function);

    if (state != DuckDBSuccess)
        return fail("sector inside function '" + name + "': registration failed, name taken or function incomplete");

    loginf << "registered sector inside function '" << name << "' sectors " << sectors.size()
           << " delta_deg " << delta_deg;

    return true;
}
