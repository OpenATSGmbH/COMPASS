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

#include "colors_edit_widget.h"

#include "color_provider.h"
#include "compass.h"
#include "data_source.h"
#include "db_context.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "logger.h"

#include <QColorDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

namespace context
{

ColorsEditWidget::ColorsEditWidget(DBContextManager& manager,
                                   std::function<void()> on_changed,
                                   QWidget* parent)
    : QWidget(parent)
    , manager_(manager)
    , on_changed_(std::move(on_changed))
{
    buildUI();
    refresh();
}

void ColorsEditWidget::buildUI()
{
    auto* main = new QVBoxLayout(this);

    // ==== Preference ====
    {
        auto* box = new QGroupBox("Preference");
        auto* row = new QHBoxLayout(box);

        light_radio_ = new QRadioButton("Light");
        dark_radio_ = new QRadioButton("Dark");

        connect(light_radio_, &QRadioButton::toggled, this,
                &ColorsEditWidget::preferenceChangedSlot);
        connect(dark_radio_, &QRadioButton::toggled, this,
                &ColorsEditWidget::preferenceChangedSlot);

        row->addWidget(light_radio_);
        row->addWidget(dark_radio_);
        row->addStretch();

        auto* hint = new QLabel(
            "Changing the preference only affects future auto-generated data source colors. "
            "Existing data source colors are kept.");
        hint->setWordWrap(true);

        auto* vbox = new QVBoxLayout();
        vbox->addLayout(row);
        vbox->addWidget(hint);
        box->setLayout(vbox);
        main->addWidget(box);
    }

    // ==== DSType Colors ====
    {
        auto* box = new QGroupBox("DSType Colors");
        auto* vbox = new QVBoxLayout(box);

        ds_type_rows_layout_ = new QVBoxLayout();
        vbox->addLayout(ds_type_rows_layout_);

        auto* reset = new QPushButton("Reset to defaults");
        reset->setIcon(QIcon());
        reset->setToolTip("Restore the default DSType color palette");
        connect(reset, &QPushButton::clicked, this,
                &ColorsEditWidget::resetDSTypeDefaultsSlot);

        auto* row = new QHBoxLayout();
        row->addStretch();
        row->addWidget(reset);
        vbox->addLayout(row);

        main->addWidget(box);
    }

    // ==== DBContent Colors ====
    {
        auto* box = new QGroupBox("DBContent Colors");
        auto* vbox = new QVBoxLayout(box);

        dbcontent_rows_layout_ = new QVBoxLayout();
        vbox->addLayout(dbcontent_rows_layout_);

        auto* reset = new QPushButton("Reset to defaults");
        reset->setIcon(QIcon());
        reset->setToolTip("Restore the default DBContent color palette");
        connect(reset, &QPushButton::clicked, this,
                &ColorsEditWidget::resetDBContentDefaultsSlot);

        auto* row = new QHBoxLayout();
        row->addStretch();
        row->addWidget(reset);
        vbox->addLayout(row);

        main->addWidget(box);
    }

    main->addStretch();
}

void ColorsEditWidget::applySwatch(QPushButton* button, const QColor& color)
{
    if (!button) return;

    if (color.isValid())
    {
        button->setStyleSheet("background-color: " + color.name() + "; color: transparent;");
        button->setText("");
    }
    else
    {
        button->setStyleSheet("");
    }
}

void ColorsEditWidget::clearLayout(QVBoxLayout* layout)
{
    while (QLayoutItem* item = layout->takeAt(0))
    {
        if (QWidget* w = item->widget())
            w->deleteLater();
        if (QLayout* l = item->layout())
            clearLayout(static_cast<QVBoxLayout*>(l));
        delete item;
    }
}

void ColorsEditWidget::refresh()
{
    if (!manager_.hasActiveContext())
    {
        light_radio_->setEnabled(false);
        dark_radio_->setEnabled(false);
        return;
    }

    light_radio_->setEnabled(true);
    dark_radio_->setEnabled(true);

    const auto& colors = manager_.activeContext().colors();

    QSignalBlocker bl(light_radio_);
    QSignalBlocker bd(dark_radio_);
    light_radio_->setChecked(colors.preference == ContextColors::Preference::Light);
    dark_radio_->setChecked(colors.preference == ContextColors::Preference::Dark);

    // DSType rows
    clearLayout(ds_type_rows_layout_);
    ds_type_buttons_.clear();
    for (const auto& ds_type : DataSource::dsTypeStrings())
    {
        auto* row = new QHBoxLayout();
        row->addWidget(new QLabel(QString::fromStdString(ds_type)));

        auto* btn = new QPushButton();
        btn->setIcon(QIcon());
        btn->setToolTip(QString::fromStdString("Color for DSType " + ds_type));
        btn->setProperty("ds_type", QString::fromStdString(ds_type));
        btn->setMinimumWidth(120);

        QColor color;
        auto it = colors.ds_type_colors.find(ds_type);
        if (it != colors.ds_type_colors.end())
            color = it->second;
        applySwatch(btn, color);
        if (!color.isValid())
            btn->setText("—");

        connect(btn, &QPushButton::clicked, this,
                &ColorsEditWidget::dsTypeColorClickedSlot);

        row->addWidget(btn);
        row->addStretch();

        ds_type_rows_layout_->addLayout(row);
        ds_type_buttons_[ds_type] = btn;
    }

    // DBContent rows — enumerate configured DBContents
    clearLayout(dbcontent_rows_layout_);
    dbcontent_buttons_.clear();

    std::vector<std::string> dbc_names;
    auto& dbcont_man = manager_.compass().dbContentManager();
    for (auto it = dbcont_man.begin(); it != dbcont_man.end(); ++it)
        dbc_names.push_back(it->first);

    // Also include any entries already present in the saved palette (e.g. "RefTraj"
    // which is not a DBContent but is used by the Geographic View palette).
    for (const auto& [name, color] : colors.dbcontent_colors)
    {
        if (std::find(dbc_names.begin(), dbc_names.end(), name) == dbc_names.end())
            dbc_names.push_back(name);
    }

    std::sort(dbc_names.begin(), dbc_names.end());

    for (const auto& name : dbc_names)
    {
        auto* row = new QHBoxLayout();
        row->addWidget(new QLabel(QString::fromStdString(name)));

        auto* btn = new QPushButton();
        btn->setIcon(QIcon());
        btn->setToolTip(QString::fromStdString("Color for DBContent " + name));
        btn->setProperty("dbcontent", QString::fromStdString(name));
        btn->setMinimumWidth(120);

        QColor color;
        auto cit = colors.dbcontent_colors.find(name);
        if (cit != colors.dbcontent_colors.end())
            color = cit->second;
        applySwatch(btn, color);
        if (!color.isValid())
            btn->setText("—");

        connect(btn, &QPushButton::clicked, this,
                &ColorsEditWidget::dbContentColorClickedSlot);

        row->addWidget(btn);
        row->addStretch();

        dbcontent_rows_layout_->addLayout(row);
        dbcontent_buttons_[name] = btn;
    }
}

void ColorsEditWidget::preferenceChangedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    // only react to the button that just became checked
    auto* radio = qobject_cast<QRadioButton*>(sender());
    if (!radio || !radio->isChecked())
        return;

    auto& ctx = manager_.activeContext();
    auto previous = ctx.colors().preference;
    auto next = light_radio_->isChecked()
                    ? ContextColors::Preference::Light
                    : ContextColors::Preference::Dark;

    if (previous == next)
        return;

    ctx.colors().preference = next;

    loginf << "context '" << ctx.name() << "' color preference "
           << (next == ContextColors::Preference::Dark ? "Dark" : "Light");

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::dsTypeColorClickedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    std::string ds_type = btn->property("ds_type").toString().toStdString();
    if (ds_type.empty()) return;

    auto& colors = manager_.activeContext().colors();
    QColor initial = colors.ds_type_colors.count(ds_type)
                         ? colors.ds_type_colors.at(ds_type)
                         : ColorProvider::defaultDSTypeColor(ds_type);

    QColor chosen = QColorDialog::getColor(initial, this,
                        QString::fromStdString("Color for DSType " + ds_type));
    if (!chosen.isValid())
        return;

    loginf << "dstype '" << ds_type << "' color " << chosen.name().toStdString();

    colors.ds_type_colors[ds_type] = chosen;
    applySwatch(btn, chosen);

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::dbContentColorClickedSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    std::string dbcontent = btn->property("dbcontent").toString().toStdString();
    if (dbcontent.empty()) return;

    auto& colors = manager_.activeContext().colors();
    QColor initial = colors.dbcontent_colors.count(dbcontent)
                         ? colors.dbcontent_colors.at(dbcontent)
                         : ColorProvider::defaultDBContentColor(dbcontent);

    QColor chosen = QColorDialog::getColor(initial, this,
                        QString::fromStdString("Color for DBContent " + dbcontent));
    if (!chosen.isValid())
        return;

    loginf << "dbcontent '" << dbcontent << "' color " << chosen.name().toStdString();

    colors.dbcontent_colors[dbcontent] = chosen;
    applySwatch(btn, chosen);

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::resetDSTypeDefaultsSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto& colors = manager_.activeContext().colors();
    colors.ds_type_colors = ColorProvider::defaultDSTypeColors();

    loginf << "reset DSType palette to defaults";

    refresh();

    if (on_changed_)
        on_changed_();
}

void ColorsEditWidget::resetDBContentDefaultsSlot()
{
    if (!manager_.hasActiveContext())
        return;

    auto& colors = manager_.activeContext().colors();
    colors.dbcontent_colors = ColorProvider::defaultDBContentColors();

    loginf << "reset DBContent palette to defaults";

    refresh();

    if (on_changed_)
        on_changed_();
}

} // namespace context
