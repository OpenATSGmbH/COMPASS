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

#include "managesectorstask.h"
#include "compass.h"
#include "evaluationmanager.h"
#include "managesectorstaskdialog.h"
#include "taskmanager.h"
#include "files.h"

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

#include "json.hpp"

#include <QMessageBox>

using namespace Utils;
using namespace nlohmann;
using namespace std;

ManageSectorsTask::ManageSectorsTask(const std::string& class_id, const std::string& instance_id,
                                     TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_manage_sectors.json")
{
    registerParameter("db_file_list", &file_list_, json::array());
    registerParameter("current_filename", &current_filename_, std::string());

    createSubConfigurables();

    vector<string> cleaned_file_list;
    // clean missing files

    for (auto& filename : file_list_.get<std::vector<string>>())
    {
        if (Files::fileExists(filename))
            cleaned_file_list.push_back(filename);
    }
    file_list_ = cleaned_file_list;

    if (!hasFile(current_filename_))
        current_filename_ = "";

    tooltip_ =
            "Allows management of sectors stored in the database. "
            "This task can not be run, but is performed using the GUI elements.";

    if (canImportFile())
        parseCurrentFile(false);
}

ManageSectorsTask::~ManageSectorsTask()
{
    file_list_.clear();
}

ManageSectorsTaskDialog* ManageSectorsTask::dialog()
{
    if (!dialog_)
    {
        dialog_.reset(new ManageSectorsTaskDialog(*this));

        connect(dialog_.get(), &ManageSectorsTaskDialog::doneSignal,
                this, &ManageSectorsTask::dialogDoneSlot);
    }

    traced_assert(dialog_);
    return dialog_.get();
}

void ManageSectorsTask::dialogDoneSlot()
{
    traced_assert(dialog_);
    dialog_->hide();

    emit COMPASS::instance().evaluationManager().sectorsChangedSignal();
}

void ManageSectorsTask::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    throw std::runtime_error("ManageSectorsTask: generateSubConfigurable: unknown class_id " + class_id);
}

bool ManageSectorsTask::hasFile(const std::string& filename) const
{
    logdbg << "filename '" << filename
           << "' file_list_ '" << file_list_.dump(2) << "'";

    vector<string> tmp_list = file_list_.get<std::vector<string>>();

    return find(tmp_list.begin(), tmp_list.end(), filename) != tmp_list.end();
}

std::vector<std::string> ManageSectorsTask::fileList() const
{
    return file_list_.get<std::vector<string>>();
}

void ManageSectorsTask::addFile(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    if (file_list_.count(filename) != 0)
        throw std::invalid_argument("ManageSectorsTask: addFile: name '" + filename +
                                    "' already in use");

    vector<string> tmp_list = file_list_.get<std::vector<string>>();
    if (find(tmp_list.begin(), tmp_list.end(), filename) == tmp_list.end())
    {
        loginf << "adding filename '" << filename << "'";

        tmp_list.push_back(filename);

        sort(tmp_list.begin(), tmp_list.end());

        file_list_ = tmp_list;
    }

    loginf << "filenames '" << file_list_.dump(2) << "'";

    current_filename_ = filename;
    parseCurrentFile(false);

    if (dialog_)
        dialog_->updateFileList();
}

void ManageSectorsTask::removeCurrentFilename()
{
    loginf << "filename '" << current_filename_ << "'";

    traced_assert(current_filename_.size());
    traced_assert(hasFile(current_filename_));

    if (file_list_.count(current_filename_) != 1)
        throw std::invalid_argument("ManageSectorsTask: removeCurrentFilename: name '" +
                                    current_filename_ + "' not in use");

    file_list_.erase(current_filename_);
    current_filename_ = "";

    parse_message_ = "";
    found_sectors_num_ = 0;

    if (dialog_)
        dialog_->updateFileList();
}

void ManageSectorsTask::removeAllFiles ()
{
    loginf;

    file_list_.clear();
    current_filename_ = "";

    parse_message_ = "";
    found_sectors_num_ = 0;

    if (dialog_)
        dialog_->updateFileList();
}

void ManageSectorsTask::currentFilename(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    current_filename_ = filename;

    if (canImportFile())
        parseCurrentFile(false);
    else
    {
        parse_message_ = "";
        found_sectors_num_ = 0;
    }
}

std::string ManageSectorsTask::parseMessage() const
{
    return parse_message_;
}

//const map<string, map<string, vector<pair<double, double>>>>& ManageSectorsTask::parsedData() const
//{
//    return data_;
//}

bool ManageSectorsTask::canImportFile()
{
    if (!current_filename_.size())
        return false;

    if (!Files::fileExists(current_filename_))
    {
        loginf << "not possible since file '"
               << current_filename_ << "'does not exist";
        return false;
    }

    return true;
}

void ManageSectorsTask::importFile (const std::string& layer_name, bool exclude, QColor color)
{
    traced_assert(canImportFile());

    if (!found_sectors_num_)
    {
        loginf << "not possible since no data found";

        QMessageBox msgBox;
        msgBox.setText("Import not possible since no data was found.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        return;
    }

    layer_name_ = layer_name;
    exclude_ = exclude;
    color_ = color;

    parseCurrentFile(true);

    QMessageBox msgBox;
    msgBox.setText(QString("Import of ")+QString::number(found_sectors_num_)+" sectors done");
    msgBox.setIcon(QMessageBox::Information);

    if (allow_user_interactions_)
        msgBox.exec();

    loginf << "done";
}


void ManageSectorsTask::parseCurrentFile (bool import)
{
    loginf << "file '" << current_filename_ << "' import " << import;

    found_sectors_num_ = 0;
    parse_message_ = "";

    GDALAllRegister();

    GDALDataset* data_set;

    data_set = (GDALDataset*) GDALOpenEx(current_filename_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL );

    if(data_set == NULL)
    {
        logwrn << "open failed";
        parse_message_ = "file '"+current_filename_+"' open failed.";
        return;
    }

    OGRLayer* layer = nullptr;

    for (int layer_cnt=0; layer_cnt < data_set->GetLayerCount(); ++layer_cnt) // OGRLayer*
    {
        layer = data_set->GetLayer(layer_cnt);
        traced_assert(layer);

        logdbg << "found sector name '" << layer->GetName() << "'";
        std::string sector_name = layer->GetName();

        OGRFeature* feature = nullptr;

        for (int feature_cnt=0; feature_cnt < layer->GetFeatureCount(); ++feature_cnt) // OGRFeature
        {
            feature = layer->GetNextFeature();

            if (!feature) // TODO solve this
            {
                logwrn << "non-feature at cnt " << feature_cnt;
                continue;
            }

            traced_assert(feature);

            logdbg << "found feature '"
                   << feature->GetDefnRef()->GetName() << "'";

            //std::string feature_name = feature->GetDefnRef()->GetName();

            OGRFeatureDefn* feature_def = layer->GetLayerDefn();
            traced_assert(feature_def);
            int field_cnt;

            for (field_cnt = 0; field_cnt < feature_def->GetFieldCount(); field_cnt++)
            {
                OGRFieldDefn* field_def = feature_def->GetFieldDefn(field_cnt);
                traced_assert(field_def);

                switch(field_def->GetType())
                {
                    case OFTInteger:
                        logdbg << "int " << feature->GetFieldAsInteger(field_cnt);
                        break;
                    case OFTInteger64:
                        logdbg << "int64 "
                               << feature->GetFieldAsInteger64(field_cnt);
                        //printf( CPL_FRMT_GIB ",", oField.GetInteger64() );
                        break;
                    case OFTReal:
                        logdbg << "double " << feature->GetFieldAsDouble(field_cnt);
                        break;
                    case OFTString:
                        logdbg << "string '"
                               << feature->GetFieldAsString(field_cnt) << "'";
                        break;
                    default:
                        logdbg << "default " << feature->GetFieldAsString(field_cnt);
                        break;
                }
            }

            OGRGeometry* geometry;

            //geometry = feature->GetGeometryRef();

            int geom_field_cnt;
            int geom_field_num;
            geom_field_num = feature->GetGeomFieldCount();

            for(geom_field_cnt = 0; geom_field_cnt < geom_field_num; geom_field_cnt ++ )
            {
                geometry = feature->GetGeomFieldRef(geom_field_cnt);

                if (!geometry)
                    continue;
                if (geometry->getGeometryType() != wkbPolygon
                    && geometry->getGeometryType() != wkbPolygon25D
                    && geometry->getGeometryType() != wkbMultiPolygon
                    && geometry->getGeometryType() != wkbMultiPolygon25D)
                {
                    loginf << "skipping unsupported geometry name "
                           << geometry->getGeometryName()
                           << " type " << geometry->getGeometryType();
                    continue;
                }

                if(wkbFlatten(geometry->getGeometryType()) == wkbPolygon)
                {
                    OGRPolygon* polygon = dynamic_cast<OGRPolygon*>(geometry);
                    traced_assert(polygon);

                    addPolygon(sector_name, *polygon, import);
                }
                else if (wkbFlatten(geometry->getGeometryType()) == wkbMultiPolygon)
                {
                    OGRMultiPolygon* multi_polygon = dynamic_cast<OGRMultiPolygon*>(geometry);
                    traced_assert(multi_polygon);

                    OGRGeometry* sub_geometry;

                    for (int poly_cnt=0; poly_cnt < multi_polygon->getNumGeometries(); ++poly_cnt)
                    {
                        sub_geometry = multi_polygon->getGeometryRef(poly_cnt);
                        traced_assert(sub_geometry);

                        if (wkbFlatten(sub_geometry->getGeometryType()) == wkbPolygon)
                        {
                            OGRPolygon* sub_polygon = dynamic_cast<OGRPolygon*>(sub_geometry);
                            traced_assert(sub_polygon);

                            addPolygon(sector_name, *sub_polygon, import);
                        }
                        else
                            logdbg << "no polygon in multipolygon found";
                    }
                }
                else
                {
                    logdbg << "no geometry found";
                }
            }
        }
    }

    GDALClose(data_set);

    if (dialog_)
        dialog_->updateParseMessage();
}

void ManageSectorsTask::addPolygon (const std::string& sector_name, OGRPolygon& polygon, bool import)
{
    logdbg << "sector_name '" << sector_name
           << "' polygon '" << polygon.getGeometryName() << "'";

    OGRLinearRing* ring = polygon.getExteriorRing();
    traced_assert(ring);

    if (import)
    {
        addLinearRing(sector_name+to_string(COMPASS::instance().evaluationManager().getMaxSectorId()+1), *ring, import);

         for (int ring_cnt=0; ring_cnt < polygon.getNumInteriorRings(); ++ring_cnt) // OGRLinearRing
         {
             ring = polygon.getInteriorRing(ring_cnt);
             traced_assert(ring);
             addLinearRing(sector_name+to_string(COMPASS::instance().evaluationManager().getMaxSectorId()+1), *ring, import);
         }
    }
    else // no eval man call during ctor
    {
        addLinearRing(sector_name, *ring, import);

         for (int ring_cnt=0; ring_cnt < polygon.getNumInteriorRings(); ++ring_cnt) // OGRLinearRing
         {
             ring = polygon.getInteriorRing(ring_cnt);
             traced_assert(ring);
             addLinearRing(sector_name, *ring, import);
         }
    }
}

void ManageSectorsTask::addLinearRing (const std::string& sector_name, OGRLinearRing& ring, bool import)
{
    logdbg << "layer '" << layer_name_ << "' sector_name '" << sector_name;

    vector<pair<double,double>> points;

    OGRPoint point;

    for (int point_cnt=0; point_cnt < ring.getNumPoints(); ++point_cnt)
    {
        ring.getPoint(point_cnt, &point);
        traced_assert(!point.IsEmpty());

        logdbg << "point lat " << point.getY()
               << " lon " << point.getX() << " z " << point.getZ();

        points.push_back({point.getY(), point.getX()});
    }

    if (points.size())
    {

        if (layer_name_.size())
            parse_message_ += "Found layer '"+layer_name_+"' sector name '"+sector_name
                    +"' num points "+to_string(points.size())+"\n";
        else
            parse_message_ += "Found sector name '"+sector_name
                    +"' num points "+to_string(points.size())+"\n";

        ++found_sectors_num_;

        if (import)
            addSector (sector_name, move(points));
    }
}

void ManageSectorsTask::addSector (const std::string& sector_name, std::vector<std::pair<double,double>> points)
{
    loginf << "layer '" << layer_name_ << "' name '" << sector_name
           << "' num points " << points.size();

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    traced_assert(!eval_man.hasSector(sector_name, layer_name_));

    loginf << "adding layer '" << layer_name_ << "' name '" << sector_name;
    eval_man.createNewSector(sector_name, layer_name_, exclude_, color_, points);
}
