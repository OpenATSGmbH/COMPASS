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

#include "sector_import_utils.h"
#include "logger.h"
#include "traced_assert.h"

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

using namespace std;

namespace sector_utils
{

static void addLinearRing(const string& sector_name, OGRLinearRing& ring,
                          vector<ImportedSector>& result)
{
    vector<pair<double, double>> points;

    OGRPoint point;
    for (int i = 0; i < ring.getNumPoints(); ++i)
    {
        ring.getPoint(i, &point);
        traced_assert(!point.IsEmpty());
        points.push_back({point.getY(), point.getX()});
    }

    if (!points.empty())
    {
        logdbg << "sector '" << sector_name << "' with " << points.size() << " points";
        result.push_back({sector_name, std::move(points)});
    }
}

static void addPolygon(const string& base_name, OGRPolygon& polygon,
                       unsigned int& running_id, vector<ImportedSector>& result)
{
    OGRLinearRing* ring = polygon.getExteriorRing();
    traced_assert(ring);

    addLinearRing(base_name + to_string(running_id++), *ring, result);

    for (int i = 0; i < polygon.getNumInteriorRings(); ++i)
    {
        ring = polygon.getInteriorRing(i);
        traced_assert(ring);
        addLinearRing(base_name + to_string(running_id++), *ring, result);
    }
}

vector<ImportedSector> parseGDALFile(const string& filepath)
{
    loginf << "parsing '" << filepath << "'";

    vector<ImportedSector> result;

    GDALAllRegister();

    GDALDataset* data_set = (GDALDataset*)GDALOpenEx(
        filepath.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

    if (!data_set)
    {
        logwrn << "open failed for '" << filepath << "'";
        return result;
    }

    unsigned int running_id = 1;

    for (int layer_cnt = 0; layer_cnt < data_set->GetLayerCount(); ++layer_cnt)
    {
        OGRLayer* layer = data_set->GetLayer(layer_cnt);
        traced_assert(layer);

        string layer_name = layer->GetName();
        logdbg << "layer '" << layer_name << "'";

        for (int feat_cnt = 0; feat_cnt < layer->GetFeatureCount(); ++feat_cnt)
        {
            OGRFeature* feature = layer->GetNextFeature();
            if (!feature)
            {
                logwrn << "null feature at index " << feat_cnt;
                continue;
            }

            for (int geom_cnt = 0; geom_cnt < feature->GetGeomFieldCount(); ++geom_cnt)
            {
                OGRGeometry* geometry = feature->GetGeomFieldRef(geom_cnt);
                if (!geometry)
                    continue;

                auto flat_type = wkbFlatten(geometry->getGeometryType());

                if (flat_type == wkbPolygon)
                {
                    OGRPolygon* polygon = dynamic_cast<OGRPolygon*>(geometry);
                    traced_assert(polygon);
                    addPolygon(layer_name, *polygon, running_id, result);
                }
                else if (flat_type == wkbMultiPolygon)
                {
                    OGRMultiPolygon* multi = dynamic_cast<OGRMultiPolygon*>(geometry);
                    traced_assert(multi);

                    for (int i = 0; i < multi->getNumGeometries(); ++i)
                    {
                        OGRGeometry* sub = multi->getGeometryRef(i);
                        traced_assert(sub);

                        if (wkbFlatten(sub->getGeometryType()) == wkbPolygon)
                        {
                            OGRPolygon* sub_polygon = dynamic_cast<OGRPolygon*>(sub);
                            traced_assert(sub_polygon);
                            addPolygon(layer_name, *sub_polygon, running_id, result);
                        }
                    }
                }
                else
                {
                    loginf << "skipping unsupported geometry '"
                           << geometry->getGeometryName() << "' type " << geometry->getGeometryType();
                }
            }
        }
    }

    GDALClose(data_set);

    loginf << "parsed " << result.size() << " sectors from '" << filepath << "'";

    return result;
}

} // namespace sector_utils
