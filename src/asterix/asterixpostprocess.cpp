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
#include "stringconv.h"
#include "number.h"
#include "json.hpp"
#include "traced_assert.h"

#include <boost/range/adaptor/reversed.hpp>

using namespace Utils;
using namespace nlohmann; //#define NDEBUG #undef NDEBUG
using namespace std;

const float tod_24h = 24 * 60 * 60;

ASTERIXPostProcess::ASTERIXPostProcess() {}

void ASTERIXPostProcess::postProcess(unsigned int category, nlohmann::json& record)
{
    record["category"] = category;

    int sac{-1};
    int sic{-1};

    if (!record.contains("010"))
    {
        logwrn << "record without item 010: '" << record.dump(4)
               << "', setting 256/256";

        record["010"]["SAC"] = 0;
        record["010"]["SIC"] = 255;
    }

    traced_assert(record.at("010").contains("SAC"));
    sac = record.at("010").at("SAC");

    traced_assert(record.at("010").contains("SIC"));
    sic = record.at("010").at("SIC");

    record["ds_id"] = Number::dsIdFrom(sac, sic);

    if (category == 1)  // CAT001 coversion hack
        postProcessCAT001(sac, sic, record);
    else if (category == 2)  // save last tods
        postProcessCAT002(sac, sic, record);
    else if (category == 10)
        postProcessCAT010(sac, sic, record);
    else if (category == 20)
        postProcessCAT020(sac, sic, record);
    else if (category == 21)
        postProcessCAT021(sac, sic, record);
    else if (category == 48)
        postProcessCAT048(sac, sic, record);
    else if (category == 62)
        postProcessCAT062(sac, sic, record);
}

void ASTERIXPostProcess::postProcessCAT001(int sac, int sic, nlohmann::json& record)
{
    // antenna 0,1 to 1,2
//    if (record.contains("020") && record.at("020").contains("ANT"))
//    {
//        nlohmann::json& item_020 = record.at("020");
//        unsigned int antenna = item_020.at("ANT");
//        item_020.at("ANT") = antenna + 1;
//    }

    // civil emergency
//    if (record.contains("070") && record.at("070").contains("Mode-3/A reply"))
//    {
//        nlohmann::json& item = record.at("070");
//        unsigned int mode3a_code = item.at("Mode-3/A reply");

//        if (mode3a_code == 7500)
//            record["civil_emergency"] = 5;
//        else if (mode3a_code == 7600)
//            record["civil_emergency"] = 6;
//        else if (mode3a_code == 7700)
//            record["civil_emergency"] = 7;
//    }

    // rdpc 0,1 to 1,2
//    if (record.contains("170") && record.at("170").contains("RDPC"))
//    {
//        nlohmann::json& item = record.at("170");
//        unsigned int value = item.at("RDPC");
//        item.at("RDPC") = value + 1;
//    }

    if (record.contains("141") && record.at("141").contains("Truncated Time of Day"))
    {
        if (sac > -1 && sic > -1)  // bingo
        {
            std::pair<unsigned int, unsigned int> sac_sic({sac, sic});

            if (cat002_last_tod_period_.count(sac_sic))
            {
                double tod = record.at("141").at("Truncated Time of Day");

                if (tod < 0 || tod >= tod_24h)
                {
                    logwrn << "impossible tod "
                           << String::timeStringFromDouble(tod);
                    record["140"]["Time-of-Day"] = nullptr;
                    return;
                }

                if (cat002_last_tod_period_.at(sac_sic) < 0 || cat002_last_tod_period_.at(sac_sic) >= tod_24h)
                {
                    logwrn << "impossible cat002 time "
                           << String::timeStringFromDouble(cat002_last_tod_period_.at(sac_sic));
                    record["140"]["Time-of-Day"] = nullptr;
                    return;
                }

                tod += cat002_last_tod_period_.at(sac_sic);

                if (tod < 0 || tod >= tod_24h)
                {
                    logwrn << "impossible corrected tod "
                           << String::timeStringFromDouble(tod);
                    record["140"]["Time-of-Day"] = nullptr;
                    return;
                }

                //  loginf << "corrected " <<
                //  String::timeStringFromDouble(record.at("140").at("Time-of-Day"))
                //      << " to " << String::timeStringFromDouble(tod)
                //     << " last update " << cat002_last_tod_period_.at(sac_sic);

                record["140"]["Time-of-Day"] = tod;
            }
            else
            {
                loginf << "removing truncated tod "
                       << String::timeStringFromDouble(record.at("141").at("Truncated Time of Day"))
                       << " since to CAT002 from sensor " << sac << "/" << sic << " is not present";
                record["140"]["Time-of-Day"] = nullptr;
            }

            //     loginf << "UGA " <<
            //     String::timeStringFromDouble(record.at("140").at("Time-of-Day"))
            //       << " sac " << sac << " sic " << sic << " cnt " <<
            //       cat002_last_tod_period_.count(sac_sic);

            //    traced_assert(record.at("140").at("Time-of-Day") > 3600.0);
        }
        else
        {
            logdbg << "skipping cat001 report without sac/sic";
            record["140"]["Time-of-Day"] = nullptr;
        }
    }
    else
    {
        if (sac > -1 && sic > -1)
        {
            std::pair<unsigned int, unsigned int> sac_sic({sac, sic});

            if (cat002_last_tod_.count(sac_sic))
            {
                double tod = cat002_last_tod_.at(sac_sic);

                traced_assert(tod >= 0 && tod <= tod_24h);
                record["140"]["Time-of-Day"] = tod;  // set tod, better than nothing

            }
            else
                logdbg << "skipping cat001 report without "
                          "truncated time of day"
                       << " or last cat002 time";
        }
        else
            logdbg << "skipping cat001 report without truncated "
                      "time of day"
                   << " or sac/sic";
    }
}

void ASTERIXPostProcess::postProcessCAT002(int sac, int sic, nlohmann::json& record)
{
    //"030": "Time of Day": 33501.4140625

    if (record.contains("030"))
    {
        if (sac > -1 && sic > -1)  // bingo
        {
            // std::pair<unsigned int, unsigned int> sac_sic ({sac, sic});
            double cat002_last_tod = record.at("030").at("Time of Day");
            double cat002_last_tod_period = 512.0 * ((int)(cat002_last_tod / 512));

            if (cat002_last_tod < 0 || cat002_last_tod > tod_24h)
            {
                logerr << "cat002_last_tod "
                       << String::timeStringFromDouble(cat002_last_tod);
                return;
            }

            if (cat002_last_tod_period < 0 || cat002_last_tod_period > tod_24h)
            {
                logerr << "cat002_last_tod_period "
                       << String::timeStringFromDouble(cat002_last_tod_period);
                return;
            }

            traced_assert(cat002_last_tod >= 0 && cat002_last_tod <= tod_24h);
            traced_assert(cat002_last_tod_period >= 0 && cat002_last_tod_period <= tod_24h);

            cat002_last_tod_period_[std::make_pair(sac, sic)] = cat002_last_tod_period;
            cat002_last_tod_[std::make_pair(sac, sic)] = cat002_last_tod;
        }
    }
}

void ASTERIXPostProcess::postProcessCAT010(int sac, int sic, nlohmann::json& record)
{
    // 500.SDP-xy (σ_xy) Covariance in two’s complement, no adjustment needed
}

void ASTERIXPostProcess::postProcessCAT020(int sac, int sic, nlohmann::json& record)
{
    // "500.SDP.rho-xy"
    if (record.contains("500") && record.at("500").contains("SDP"))
    {
        nlohmann::json& item_500_stp = record.at("500").at("SDP");
        traced_assert(item_500_stp.contains("rho-xy"));
        traced_assert(item_500_stp.contains("sigma-x"));
        traced_assert(item_500_stp.contains("sigma-y"));

        // Covariance from Correlation Coefficient CovXY​=ρXY​⋅σX​⋅σY
        double rho_xy = item_500_stp.at("rho-xy");
        double sigma_x = item_500_stp.at("sigma-x");
        double sigma_y = item_500_stp.at("sigma-y");

        item_500_stp.at("rho-xy") = rho_xy * sigma_x * sigma_y;
    }

    // "REF.PA.SDC.COV-XY (Covariance Component)"
    if (record.contains("REF") && record.at("REF").contains("PA")
        && record.at("REF").at("PA").contains("SDC"))
    {
        nlohmann::json& item_ref_pa = record.at("REF").at("PA").at("SDC");
        traced_assert(item_ref_pa.contains("COV-XY (Covariance Component)"));

        // XY covariance component = sign {Cov(X,Y)} * sqrt {abs [Cov (X,Y)]}

        double cov_xy = item_ref_pa.at("COV-XY (Covariance Component)");

        cov_xy = (cov_xy < 0) ? -std::pow(cov_xy, 2) : std::pow(cov_xy, 2);

        item_ref_pa.at("COV-XY (Covariance Component)") = cov_xy;
    }


    // altitude capability
//    if (record.contains("230") && record.at("230").contains("ARC"))
//    {
//        nlohmann::json& item_230 = record.at("230");
//        unsigned int arc = item_230.at("ARC");
//        if (arc == 0)
//            item_230["ARC_ft"] = 100.0;
//        else if (arc == 1)
//            item_230["ARC_ft"] = 25.0;
//    }

    // 400 contributing receivers hack

    if (record.contains("400") && record.at("400").contains("Contributing Receivers"))
    {
        std::vector<unsigned int> ru_indexes; // collect them all

        nlohmann::json& contr_rvs = record.at("400").at("Contributing Receivers");
        traced_assert(contr_rvs.is_array());

        logdbg << "processing '" << contr_rvs.dump() << "'";

        unsigned int prev_bit_cnt = 0;

        for (const json& it : boost::adaptors::reverse(contr_rvs.get_ref<json::array_t&>()))
        {
            traced_assert(it.contains("RUx"));

            logdbg << "processing '" << it.at("RUx").dump() << "'";

            unsigned int rux_bits = it.at("RUx");
            traced_assert(rux_bits < 256);

            for (unsigned int current_bit_cnt = 0; current_bit_cnt < 8; ++current_bit_cnt)
            {
                bool current_bit = rux_bits & (0x1 << current_bit_cnt);

                if (current_bit)
                    ru_indexes.push_back(prev_bit_cnt + current_bit_cnt + 1); // count starts at 1 in ASTERIX spec
            }

            prev_bit_cnt += 8;
        }

        std::vector<std::map<std::string, json>> new_contrib_rus;

        for (auto& contrib_ru_index : ru_indexes)
            new_contrib_rus.push_back({{"RUx", contrib_ru_index}});

        record.at("400").at("Contributing Receivers") = new_contrib_rus;

        logdbg << "result '" << record.at("400").at("Contributing Receivers").dump() << "'";
    }
}

void ASTERIXPostProcess::postProcessCAT021(int sac, int sic, nlohmann::json& record)
{
//    if (record.contains("150"))  // true airspeed
//    {
//        json& air_speed_item = record.at("150");
//        traced_assert(air_speed_item.contains("IM"));
//        traced_assert(air_speed_item.contains("Air Speed"));

//        bool mach = air_speed_item.at("IM") == 1;
//        double airspeed = air_speed_item.at("Air Speed");

//        if (mach)
//        {
//            air_speed_item["Air Speed [knots]"] = airspeed * 0.001 * 666.739;  // lsb, mach 2 kn
//            air_speed_item["Air Speed [mach]"] = airspeed * 0.001;             // lsb
//        }
//        else
//        {
//            air_speed_item["Air Speed [knots]"] = airspeed * 3600.0;   // nm/s 2 kn
//            air_speed_item["Air Speed [mach]"] = airspeed * 0.185205;  // lsb, nm/s 2 mach
//        }
//    }

    // altitude capability
//    if (record.contains("040") && record.at("040").contains("ARC"))
//    {
//        nlohmann::json& item = record.at("040");
//        unsigned int arc = item.at("ARC");
//        if (arc == 0)
//            item["ARC_ft"] = 25.0;
//        else if (arc == 1)
//            item["ARC_ft"] = 100.0;
//    }

    // link technology
//    if (record.contains("210") && record.at("210").contains("LTT"))
//    {
//        nlohmann::json& item = record.at("210");
//        unsigned int ltt = item.at("LTT");
//        // = 0 Other
//        if (ltt == 0)
//        {
//            item["LTT_OTHER"] = "Y";
//            item["LTT_UAT"] = "N";
//            item["LTT_MDS"] = "N";
//            item["LTT_VDL"] = "N";
//        }
//        // = 1 UAT
//        if (ltt == 0)
//        {
//            item["LTT_OTHER"] = "N";
//            item["LTT_UAT"] = "Y";
//            item["LTT_MDS"] = "N";
//            item["LTT_VDL"] = "N";
//        }
//        // = 2 1090 ES
//        else if (ltt == 2)
//        {
//            item["LTT_OTHER"] = "N";
//            item["LTT_UAT"] = "N";
//            item["LTT_MDS"] = "Y";
//            item["LTT_VDL"] = "N";
//        }
//        // = 3 VDL 4
//        else if (ltt == 3)
//        {
//            item["LTT_OTHER"] = "N";
//            item["LTT_UAT"] = "N";
//            item["LTT_MDS"] = "N";
//            item["LTT_VDL"] = "Y";
//        }
//    }

    // ecat str
//    if (record.contains("020") && record.at("020").contains("ECAT"))
//    {
//        nlohmann::json& item = record.at("020");
//        unsigned int ecat = item.at("ECAT");

//        //# 0 = No ADS-B Emitter Category Information
//        //# value record '0' db 'NO_INFO': 20672
//        if (ecat == 0)
//            item["ECAT_str"] = "NO_INFO";

//        //# 1 = light aircraft <= 15500 lbs
//        //# value record '1' db 'LIGHT_AIRCRAFT': 2479
//        else if (ecat == 1)
//            item["ECAT_str"] = "LIGHT_AIRCRAFT";

//        //# 2 = 15500 lbs < small aircraft <75000 lbs
//        //# value record '2' db 'SMALL_AIRCRAFT': 1941
//        else if (ecat == 2)
//            item["ECAT_str"] = "SMALL_AIRCRAFT";

//        //# 3 = 75000 lbs < medium a/c < 300000 lbs
//        //# value record '3' db 'MEDIUM_AIRCRAFT': 44098
//        else if (ecat == 3)
//            item["ECAT_str"] = "MEDIUM_AIRCRAFT";

//        //# 4 = High Vortex Large
//        //# value record '4' db 'HIGH_VORTEX_LARGE': 7232
//        else if (ecat == 4)
//            item["ECAT_str"] = "HIGH_VORTEX_LARGE";

//        //# 5 = 300000 lbs <= heavy aircraft
//        //# value record '5' db 'HEAVY_AIRCRAFT': 56392
//        else if (ecat == 5)
//            item["ECAT_str"] = "HEAVY_AIRCRAFT";

//        //# 6 = highly manoeuvrable (5g acceleration capability) and high speed (>400 knots cruise)
//        else if (ecat == 6)
//            item["ECAT_str"] = "HIGHLY_MANOEUVRABLE";

//        //# 7 to 9 = reserved
//        // elif ecat in (7, 8, 9):
//        //    return None  # ?

//        //# 10 = rotocraft
//        // elif ecat == 10:
//        //    return 'ROTOCRAFT'  # ?
//        else if (ecat == 0)
//            item["ECAT_str"] = "NO_INFO";

//        //# 11 = glider / sailplane
//        else if (ecat == 11)
//            item["ECAT_str"] = "GLIDER";

//        //# 12 = lighter-than-air
//        // elif ecat == 12:
//        //    return 'LIGHTER_THAN_AIR'  # ?
//        else if (ecat == 12)
//            item["ECAT_str"] = "LIGHTER_THAN_AIR";

//        //# 13 = unmanned aerial vehicle
//        else if (ecat == 13)
//            item["ECAT_str"] = "UNMANNED";

//        //# 14 = space / transatmospheric vehicle
//        else if (ecat == 14)
//            item["ECAT_str"] = "SPACE_VEHICLE";

//        //# 15 = ultralight / handglider / paraglider
//        else if (ecat == 15)
//            item["ECAT_str"] = "ULTRALIGHT";

//        //# 16 = parachutist / skydiver
//        else if (ecat == 16)
//            item["ECAT_str"] = "SKYDIVER";

//        //# 17 to 19 = reserved
//        // elif ecat in (17, 18, 19):
//        //    return None  # ?

//        //# 20 = surface emergency vehicle
//        //# value record '20' db 'SURF_EMERGENCY': 286
//        else if (ecat == 20)
//            item["ECAT_str"] = "SURF_EMERGENCY";

//        //# 21 = surface service vehicle
//        //# value record '21' db 'SURF_SERVICE': 12773
//        else if (ecat == 21)
//            item["ECAT_str"] = "SURF_SERVICE";

//        //# 22 = fixed ground or tethered obstruction
//        else if (ecat == 22)
//            item["ECAT_str"] = "GROUND_OBSTRUCTION";

//        //# 23 = cluster obstacle
//        // elif ecat == 23:
//        //    return 'CLUSTER_OBSTACLE'  # ?
//        else if (ecat == 0)
//            item["ECAT_str"] = "NO_INFO";

//        //# 24 = line obstacle
//        else if (ecat == 24)
//            item["ECAT_str"] = "LINE_OBSTACLE";
//    }

    // surveillance status str
//    if (record.contains("020") && record.at("020").contains("SS"))
//    {
//        nlohmann::json& item = record.at("020");
//        unsigned int ss = item.at("SS");

//        //        # 0 No condition reported
//        if (ss == 0)
//            item["SS_str"] = "NO_CONDITION";
//        //        # = 1 Permanent Alert (Emergency condition)
//        else if (ss == 1)
//            item["SS_str"] = "PERMANENT_ALERT";
//        //        # = 2 Temporary Alert (change in Mode 3/A Code other than emergency)
//        else if (ss == 2)
//            item["SS_str"] = "TEMPORARY_ALERT";
//        //        # = 3 SPI set
//        else if (ss == 3)
//            item["SS_str"] = "SPI_SET";
//    }
}

void ASTERIXPostProcess::postProcessCAT048(int sac, int sic, nlohmann::json& record)
{
    // altitude capability
//    if (record.contains("230") && record.at("230").contains("ARC"))
//    {
//        nlohmann::json& item_230 = record.at("230");
//        unsigned int arc = item_230.at("ARC");
//        if (arc == 0)
//            item_230["ARC_ft"] = 100.0;
//        else if (arc == 1)
//            item_230["ARC_ft"] = 25.0;
//    }

    // civil emergency
//    if (record.contains("070") && record.at("070").contains("Mode-3/A reply"))
//    {
//        nlohmann::json& item = record.at("070");
//        unsigned int mode3a_code = item.at("Mode-3/A reply");

//        if (mode3a_code == 7500)
//            record["civil_emergency"] = 5;
//        else if (mode3a_code == 7600)
//            record["civil_emergency"] = 6;
//        else if (mode3a_code == 7700)
//            record["civil_emergency"] = 7;
//    }

    // ground bit
//    if (record.contains("230") && record.at("230").contains("STAT"))
//    {
//        nlohmann::json& item = record.at("230");
//        unsigned int stat = item.at("STAT");

//        //        # = 0 No alert, no SPI, aircraft airborne
//        //        if stat == 0:
//        //            return "N"
//        if (stat == 0)
//            record["ground_bit"] = "N";
//        //        # = 1 No alert, no SPI, aircraft on ground
//        //        if stat == 1:
//        //            return "Y"
//        else if (stat == 1)
//            record["ground_bit"] = "Y";
//        //        # = 2 Alert, no SPI, aircraft airborne
//        //        if stat == 2:
//        //            return "N"
//        else if (stat == 2)
//            record["ground_bit"] = "N";
//        //        # = 3 Alert, no SPI, aircraft on ground
//        //        if stat == 3:
//        //            return "Y"
//        else if (stat == 3)
//            record["ground_bit"] = "Y";
//    }

    // mode4 friendly
//    if (record.contains("020") && record.at("020").contains("FOE/FRI"))
//    {
//        nlohmann::json& item = record.at("020");
//        unsigned int foefrie = item.at("FOE/FRI");

//        //#Mode-4 interrorgation type:
//        //# - = no interrogation, 0 No Mode 4 interrogation
//        // if frifoe == 0:
//        //    return '-'
//        if (foefrie == 0)
//            record["mode4_friendly"] = "N";

//        //# F = Friendly target, 1 Friendly target
//        // if frifoe == 1:
//        //    return 'F'
//        else if (foefrie == 1)
//            record["mode4_friendly"] = "F";

//        //# U = Unknown Target, 2 Unknown target
//        // if frifoe == 2:
//        //    return 'U'
//        else if (foefrie == 2)
//            record["mode4_friendly"] = "U";

//        //# N = No Reply, 3 No reply
//        // if frifoe == 3:
//        //    return "N"
//        else if (foefrie == 3)
//            record["mode4_friendly"] = "N";
//    }

    // rdp chain 0,1 to 1,2
//    if (record.contains("020") && record.at("020").contains("RDP"))
//    {
//        nlohmann::json& item = record.at("020");
//        unsigned int value = item.at("RDP");
//        item.at("RDP") = value + 1;
//    }

    // track climb desc mode
//    if (record.contains("170") && record.at("170").contains("CDM"))
//    {
//        nlohmann::json& item = record.at("170");
//        unsigned int cdm = item.at("CDM");

//        //# value record '0' db 'M': 4117 M = Maintaining
//        // if cdm == 0:
//        //    return 'M'
//        if (cdm == 0)
//            record["track_climb_desc_mode"] = "M";
//        //# value record '1' db 'C': 1133 C = Climbing
//        // if cdm == 1:
//        //    return 'C'
//        else if (cdm == 1)
//            record["track_climb_desc_mode"] = "C";
//        //# value record '2' db 'D': 330 D = Descending
//        // if cdm == 2:
//        //    return 'D'
//        else if (cdm == 2)
//            record["track_climb_desc_mode"] = "D";
//        //# value record '3' db 'I': 569 I = Invalid
//        // if cdm == 3:
//        //    return 'I'
//        else if (cdm == 3)
//            record["track_climb_desc_mode"] = "I";
//    }

    // report type
//    if (record.contains("161") && record.at("161").contains("TRACK NUMBER"))
//        record["report_type"] = 1;
//    else
//        record["report_type"] = 0;
}

void ASTERIXPostProcess::postProcessCAT062(int sac, int sic, nlohmann::json& record)
{
    //"500.COV.COV (XY Covariance Component)"


    if (record.contains("500") && record.at("500").contains("COV")
        && record.at("500").at("COV").contains("COV (XY Covariance Component)"))
    {
        nlohmann::json& item_500_cov = record.at("500").at("COV").at("COV (XY Covariance Component)");

        // XY covariance component = sign {Cov(X,Y)} * sqrt {abs [Cov (X,Y)]}
        // Cov(X,Y) = sign {Cov(X,Y)} * XY covariance component^2

        double cov_xy = item_500_cov;

        cov_xy = (cov_xy < 0) ? -std::pow(cov_xy, 2) : std::pow(cov_xy, 2);

        item_500_cov = cov_xy;
    }

}
