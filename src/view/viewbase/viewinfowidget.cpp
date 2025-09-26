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

#include "viewinfowidget.h"
#include "view.h"

#include <QGridLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpacerItem>

/**
*/
ViewInfoWidget::ViewInfoWidget(View* view, QWidget* parent) 
:   QWidget(parent)
,   view_  (view  )
{
    traced_assert(view_);

    setObjectName("infowidget");

    layout_ = new QVBoxLayout;
    setLayout(layout_);

    reinit();
}

/**
*/
bool ViewInfoWidget::hasItems() const
{
    return num_items_ > 0;
}

/**
*/
void ViewInfoWidget::reinit()
{
    if (widget_ && num_items_ == 0)
        return;

    if (widget_)
        delete widget_;

    widget_ = new QWidget;
    auto widget_layout = new QVBoxLayout;
    widget_layout->setContentsMargins(0, 0, 0, 0);
    widget_layout->setSpacing(0);
    widget_->setLayout(widget_layout);

    info_layout_ = new QGridLayout;

    widget_layout->addLayout(info_layout_);
    //widget_layout->addStretch(1);

    info_layout_->setSpacing(6);

    layout_->addWidget(widget_);

    num_items_ = 0;
    displayed_infos_.clear();
}

/**
*/
void ViewInfoWidget::clear()
{
    reinit();
}

/**
*/
void ViewInfoWidget::addLine(const std::string& id, 
                             const std::string& name, 
                             const std::string& value,
                             bool is_section,
                             Style name_style,
                             Style value_style)
{
    auto label_name  = new QLabel(QString::fromStdString(name ));
    auto label_value = new QLabel(QString::fromStdString(value));

    QFont f_name  = label_name->font();
    QFont f_value = label_name->font();

    auto configFont = [ & ] (QFont& f, Style s)
    {
        if (s == Style::Bold)
            f.setBold(true);
        else if (s == Style::Italic)
            f.setItalic(true);
    };

    configFont(f_name , name_style );
    configFont(f_value, value_style);
    
    label_name->setFont(f_name);
    label_value->setFont(f_value);

    label_name->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    label_value->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    int r = info_layout_->rowCount();

    auto layout_name = new QHBoxLayout;
    layout_name->setContentsMargins(0, 0, 0, 0);
    layout_name->setSpacing(0);

    layout_name->addSpacerItem(new QSpacerItem(is_section ? 1 : 10, 1, QSizePolicy::Fixed, QSizePolicy::Fixed));
    layout_name->addWidget(label_name);

    info_layout_->addLayout(layout_name , r, 0);
    info_layout_->addItem(new QSpacerItem(10, 1, QSizePolicy::Fixed, QSizePolicy::Fixed) , r, 1);
    info_layout_->addWidget(label_value, r, 2);

    DisplayedInfo di;
    di.id          = id;
    di.label_name  = label_name;
    di.label_value = label_value;

    displayed_infos_.push_back(di);

    ++num_items_;
}

/**
 */
void ViewInfoWidget::addInfo(const std::string& id,
                             const std::string& name, 
                             const std::string& value,
                             bool value_italic)
{
    addInfos(ViewInfos().addInfo(id, name, value, value_italic));
}

/**
 */
void ViewInfoWidget::addSection(const std::string& name)
{
    addInfos(ViewInfos().addSection(name));
}

/**
*/
void ViewInfoWidget::addSpace()
{
    addInfos(ViewInfos().addSpace());
}

/**
*/
void ViewInfoWidget::addInfos(const ViewInfos& infos)
{
    for (const auto& i : infos.infos())
        addLine(i.id, i.name, i.value, i.is_section, i.name_style, i.value_style);
}

/**
*/
void ViewInfoWidget::updateInfos()
{
    clear();

    auto infos = view_->viewInfos();

    //loginf << "adding " << infos.numInfos() 
    //      << " info(s) in " << infos.numSections() << " section(s)"
    //      << " to " << view_->instanceId();

    addInfos(infos);
}

/**
*/
void ViewInfoWidget::loadingStarted()
{
    clear();
}

/**
*/
void ViewInfoWidget::loadingDone()
{
    //@TODO: needed?
    updateInfos();
}

/**
*/
void ViewInfoWidget::redrawStarted()
{
    clear();
}

/**
*/
void ViewInfoWidget::redrawDone()
{
    //@TODO: needed?
    updateInfos();
}

/**
*/
void ViewInfoWidget::appModeSwitch(AppMode app_mode)
{
    //@TODO?
}

/**
*/
void ViewInfoWidget::configChanged()
{
    updateInfos();
}

/**
*/
void ViewInfoWidget::onDisplayChange()
{
    updateInfos();
}

/**
*/
nlohmann::json ViewInfoWidget::viewInfoJSON() const
{
    nlohmann::json j;

    for (const auto& di : displayed_infos_)
        if (!di.id.empty() && di.label_value)
            j[ di.id ] = di.label_value->text().toStdString();

    return j;
}
