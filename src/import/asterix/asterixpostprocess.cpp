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
    // jASTERIX now handles CAT001 truncated time fix and writes "140.Time-of-Day" directly
    // if (category == 1)
    //     postProcessFlatCAT001(d, num_records);
    // else if (category == 2)
    //     postProcessFlatCAT002(d, num_records);
    // else
    if (category == 20)
        postProcessFlatCAT020(d, num_records);
    else if (category == 62)
        postProcessFlatCAT062(d, num_records);
}

// jASTERIX now handles CAT001 truncated time fix and writes "140.Time-of-Day" directly,
// so postProcessFlatCAT001/CAT002 are no longer needed.
//
// void ASTERIXPostProcess::postProcessFlatCAT001(nlohmann::json& d, size_t num_records)
// {
//     const json* sac_arr = d.contains("010.SAC") ? &d.at("010.SAC") : nullptr;
//     const json* sic_arr = d.contains("010.SIC") ? &d.at("010.SIC") : nullptr;
//     const json* trunc_tod_arr = d.contains("141.Truncated Time of Day") ? &d.at("141.Truncated Time of Day") : nullptr;
//
//     // ensure 140.Time-of-Day array exists
//     if (!d.contains("140.Time-of-Day"))
//     {
//         d["140.Time-of-Day"] = json::array();
//         d["140.Time-of-Day"].get_ref<json::array_t&>().resize(num_records, nullptr);
//     }
//     json& tod_out = d["140.Time-of-Day"];
//     if (tod_out.size() < num_records)
//         tod_out.get_ref<json::array_t&>().resize(num_records, nullptr);
//
//     for (size_t i = 0; i < num_records; ++i)
//     {
//         int sac = -1, sic = -1;
//         if (sac_arr && i < sac_arr->size() && !(*sac_arr)[i].is_null())
//             sac = (*sac_arr)[i].get<int>();
//         if (sic_arr && i < sic_arr->size() && !(*sic_arr)[i].is_null())
//             sic = (*sic_arr)[i].get<int>();
//
//         bool has_trunc = trunc_tod_arr && i < trunc_tod_arr->size() && !(*trunc_tod_arr)[i].is_null();
//
//         if (has_trunc)
//         {
//             if (sac > -1 && sic > -1)
//             {
//                 std::pair<unsigned int, unsigned int> sac_sic({(unsigned int)sac, (unsigned int)sic});
//
//                 if (cat002_last_tod_period_.count(sac_sic))
//                 {
//                     double tod = (*trunc_tod_arr)[i].get<double>();
//
//                     if (tod < 0 || tod >= tod_24h)
//                     {
//                         logwrn << "impossible tod " << String::timeStringFromDouble(tod);
//                         tod_out[i] = nullptr;
//                         continue;
//                     }
//
//                     if (cat002_last_tod_period_.at(sac_sic) < 0 || cat002_last_tod_period_.at(sac_sic) >= tod_24h)
//                     {
//                         logwrn << "impossible cat002 time "
//                                << String::timeStringFromDouble(cat002_last_tod_period_.at(sac_sic));
//                         tod_out[i] = nullptr;
//                         continue;
//                     }
//
//                     tod += cat002_last_tod_period_.at(sac_sic);
//
//                     if (tod < 0 || tod >= tod_24h)
//                     {
//                         logwrn << "impossible corrected tod " << String::timeStringFromDouble(tod);
//                         tod_out[i] = nullptr;
//                         continue;
//                     }
//
//                     tod_out[i] = tod;
//                 }
//                 else
//                 {
//                     loginf << "removing truncated tod since no CAT002 from sensor "
//                            << sac << "/" << sic << " is present";
//                     tod_out[i] = nullptr;
//                 }
//             }
//             else
//             {
//                 logdbg << "skipping cat001 report without sac/sic";
//                 tod_out[i] = nullptr;
//             }
//         }
//         else
//         {
//             if (sac > -1 && sic > -1)
//             {
//                 std::pair<unsigned int, unsigned int> sac_sic({(unsigned int)sac, (unsigned int)sic});
//
//                 if (cat002_last_tod_.count(sac_sic))
//                 {
//                     double tod = cat002_last_tod_.at(sac_sic);
//                     traced_assert(tod >= 0 && tod <= tod_24h);
//                     tod_out[i] = tod;
//                 }
//                 else
//                     logdbg << "skipping cat001 report without truncated time of day or last cat002 time";
//             }
//             else
//                 logdbg << "skipping cat001 report without truncated time of day or sac/sic";
//         }
//     }
// }
//
// void ASTERIXPostProcess::postProcessFlatCAT002(nlohmann::json& d, size_t num_records)
// {
//     const json* sac_arr = d.contains("010.SAC") ? &d.at("010.SAC") : nullptr;
//     const json* sic_arr = d.contains("010.SIC") ? &d.at("010.SIC") : nullptr;
//     const json* tod_arr = d.contains("030.Time of Day") ? &d.at("030.Time of Day") : nullptr;
//
//     if (!tod_arr || !sac_arr || !sic_arr)
//         return;
//
//     for (size_t i = 0; i < num_records; ++i)
//     {
//         if (i >= tod_arr->size() || (*tod_arr)[i].is_null())
//             continue;
//         if (i >= sac_arr->size() || (*sac_arr)[i].is_null())
//             continue;
//         if (i >= sic_arr->size() || (*sic_arr)[i].is_null())
//             continue;
//
//         int sac = (*sac_arr)[i].get<int>();
//         int sic = (*sic_arr)[i].get<int>();
//
//         if (sac < 0 || sic < 0)
//             continue;
//
//         double cat002_last_tod = (*tod_arr)[i].get<double>();
//         double cat002_last_tod_period = 512.0 * ((int)(cat002_last_tod / 512));
//
//         if (cat002_last_tod < 0 || cat002_last_tod > tod_24h)
//         {
//             logerr << "cat002_last_tod " << String::timeStringFromDouble(cat002_last_tod);
//             continue;
//         }
//
//         if (cat002_last_tod_period < 0 || cat002_last_tod_period > tod_24h)
//         {
//             logerr << "cat002_last_tod_period " << String::timeStringFromDouble(cat002_last_tod_period);
//             continue;
//         }
//
//         cat002_last_tod_period_[std::make_pair((unsigned int)sac, (unsigned int)sic)] = cat002_last_tod_period;
//         cat002_last_tod_[std::make_pair((unsigned int)sac, (unsigned int)sic)] = cat002_last_tod;
//     }
// }

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
