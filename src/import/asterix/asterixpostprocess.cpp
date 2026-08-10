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

#include "asterixpostprocess.h"

#include "logger.h"
#include "number.h"
#include "json.hpp"
#include "traced_assert.h"

using namespace Utils;
using namespace nlohmann;
using namespace std;

ASTERIXPostProcess::ASTERIXPostProcess() {}

void ASTERIXPostProcess::postProcessFlat(unsigned int category, unsigned int line_id,
                                         nlohmann::json& d, size_t num_records)
{
    if (num_records == 0)
        return;

    // add category array
    d["category"] = json::array();
    json& cat_arr = d["category"];
    cat_arr.get_ref<json::array_t&>().resize(num_records, category);

    // add line_id array
    d["line_id"] = json::array();
    json& lid_arr = d["line_id"];
    lid_arr.get_ref<json::array_t&>().resize(num_records, line_id);

    // compute ds_id from 010.SAC / 010.SIC
    {
        const json* sac_arr_ptr = d.contains("010.SAC") ? &d.at("010.SAC") : nullptr;
        const json* sic_arr_ptr = d.contains("010.SIC") ? &d.at("010.SIC") : nullptr;

        d["ds_id"] = json::array();
        json& ds_id_arr = d["ds_id"];
        ds_id_arr.get_ref<json::array_t&>().resize(num_records, nullptr);

        for (size_t i = 0; i < num_records; ++i)
        {
            int sac = -1;
            int sic = -1;

            if (sac_arr_ptr && i < sac_arr_ptr->size() && !(*sac_arr_ptr)[i].is_null())
                sac = (*sac_arr_ptr)[i].get<int>();
            if (sic_arr_ptr && i < sic_arr_ptr->size() && !(*sic_arr_ptr)[i].is_null())
                sic = (*sic_arr_ptr)[i].get<int>();

            if (sac < 0 || sic < 0)
            {
                logwrn << "record " << i << " without SAC/SIC, setting 0/255";
                sac = 0;
                sic = 255;
            }

            ds_id_arr[i] = Number::dsIdFrom(sac, sic);
        }
    }

    // category-specific post-processing
    // (CAT001 truncated time reconstruction is done by jASTERIX in flat mode,
    // which writes "140.Time-of-Day" directly)
    if (category == 20)
        postProcessFlatCAT020(d, num_records);
    else if (category == 62)
        postProcessFlatCAT062(d, num_records);
}

void ASTERIXPostProcess::postProcessFlatCAT020(nlohmann::json& d, size_t num_records)
{
    // "500.SDP.rho-xy" covariance correction: rho-xy = rho-xy * sigma-x * sigma-y
    if (d.contains("500.SDP.rho-xy") && d.contains("500.SDP.sigma-x") && d.contains("500.SDP.sigma-y"))
    {
        json& rho_arr = d.at("500.SDP.rho-xy");
        const json& sx_arr = d.at("500.SDP.sigma-x");
        const json& sy_arr = d.at("500.SDP.sigma-y");

        for (size_t i = 0; i < rho_arr.size() && i < num_records; ++i)
        {
            if (rho_arr[i].is_null() || i >= sx_arr.size() || sx_arr[i].is_null()
                || i >= sy_arr.size() || sy_arr[i].is_null())
                continue;

            double rho_xy = rho_arr[i].get<double>();
            double sigma_x = sx_arr[i].get<double>();
            double sigma_y = sy_arr[i].get<double>();

            rho_arr[i] = rho_xy * sigma_x * sigma_y;
        }
    }

    // "REF.PA.SDC.COV-XY (Covariance Component)" correction: sign * square
    const std::string cov_key = "REF.PA.SDC.COV-XY (Covariance Component)";
    if (d.contains(cov_key))
    {
        json& cov_arr = d.at(cov_key);

        for (size_t i = 0; i < cov_arr.size() && i < num_records; ++i)
        {
            if (cov_arr[i].is_null())
                continue;

            double cov_xy = cov_arr[i].get<double>();
            cov_arr[i] = (cov_xy < 0) ? -std::pow(cov_xy, 2) : std::pow(cov_xy, 2);
        }
    }

    // 400 contributing receivers: per-record array of raw bitmap bytes (each byte is an
    // 8-RU bitmap; the array is in spec order, last byte = lowest RUs), decoded in place
    // into a list of 1-based RU indexes for set bits
    if (d.contains("400.Contributing Receivers.RUx"))
    {
        json& rux_arr = d.at("400.Contributing Receivers.RUx");

        for (size_t i = 0; i < rux_arr.size() && i < num_records; ++i)
        {
            json& rec = rux_arr[i];
            if (rec.is_null() || !rec.is_array() || rec.empty())
                continue;

            std::vector<unsigned int> ru_indexes;
            unsigned int prev_bit_cnt = 0;

            for (auto it = rec.crbegin(); it != rec.crend(); ++it)
            {
                traced_assert(it->is_number_unsigned());
                unsigned int rux_bits = it->get<unsigned int>();
                traced_assert(rux_bits < 256);

                for (unsigned int bit = 0; bit < 8; ++bit)
                {
                    if (rux_bits & (0x1u << bit))
                        ru_indexes.push_back(prev_bit_cnt + bit + 1); // 1-based per ASTERIX
                }
                prev_bit_cnt += 8;
            }

            rec = std::move(ru_indexes);
        }
    }
}

void ASTERIXPostProcess::postProcessFlatCAT062(nlohmann::json& d, size_t num_records)
{
    // "500.COV.COV (XY Covariance Component)" correction: sign * square
    const std::string cov_key = "500.COV.COV (XY Covariance Component)";
    if (d.contains(cov_key))
    {
        json& cov_arr = d.at(cov_key);

        for (size_t i = 0; i < cov_arr.size() && i < num_records; ++i)
        {
            if (cov_arr[i].is_null())
                continue;

            double cov_xy = cov_arr[i].get<double>();
            cov_arr[i] = (cov_xy < 0) ? -std::pow(cov_xy, 2) : std::pow(cov_xy, 2);
        }
    }
}
