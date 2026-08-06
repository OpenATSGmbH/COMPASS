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

#include "asterixconfigwidget.h"
#include "asterixeditioncombobox.h"
#include "asterixrefeditioncombobox.h"
#include "asterixspfeditioncombobox.h"
#include "db_context_manager.h"
#include "files.h"
#include "logger.h"
#include "stringconv.h"
#include "traced_assert.h"

#include <jasterix/category.h>
#include <jasterix/edition.h>
#include <jasterix/refedition.h>

#include <QCheckBox>
#include <QDesktopServices>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>

using namespace std;
using namespace Utils;
using namespace jASTERIX;

ASTERIXConfigWidget::ASTERIXConfigWidget(context::DBContextManager& ctx_mgr,
                                         DecodeGetter decode_getter,
                                         DecodeSetter decode_setter,
                                         QWidget* parent)
    : QWidget(parent)
    , ctx_mgr_(ctx_mgr)
    , decode_getter_(std::move(decode_getter))
    , decode_setter_(std::move(decode_setter))
    , decode_editable_(decode_getter_ && decode_setter_)
{
    initjASTERIX();

    main_layout_ = new QVBoxLayout();

    categories_grid_ = new QGridLayout();
    updateCategories();

    main_layout_->addLayout(categories_grid_);

    if (decode_editable_)
    {
        QHBoxLayout* select_layout = new QHBoxLayout();

        QPushButton* select_all_button = new QPushButton("Select All");
        select_all_button->setObjectName("asterix_select_all_button");
        select_all_button->setIcon(QIcon());
        select_all_button->setToolTip("Enable decoding of all categories");
        connect(select_all_button, &QPushButton::clicked,
                this, [this]() { setAllCategoriesDecoded(true); });
        select_layout->addWidget(select_all_button);

        QPushButton* select_none_button = new QPushButton("Select None");
        select_none_button->setObjectName("asterix_select_none_button");
        select_none_button->setIcon(QIcon());
        select_none_button->setToolTip("Disable decoding of all categories");
        connect(select_none_button, &QPushButton::clicked,
                this, [this]() { setAllCategoriesDecoded(false); });
        select_layout->addWidget(select_none_button);

        select_layout->addStretch(1);

        main_layout_->addLayout(select_layout);
    }

    main_layout_->addStretch();

    setLayout(main_layout_);
}

ASTERIXConfigWidget::~ASTERIXConfigWidget() {}

void ASTERIXConfigWidget::initjASTERIX()
{
    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    logdbg << "jasterix definition path '" << jasterix_definition_path << "'";
    traced_assert(Files::directoryExists(jasterix_definition_path));

    jasterix_ = std::make_shared<jASTERIX::jASTERIX>(jasterix_definition_path, false, false, true);
}

void ASTERIXConfigWidget::updateSlot()
{
    loginf;
    updateCategories();
}

void ASTERIXConfigWidget::updateCategories()
{
    traced_assert(categories_grid_);

    QLayoutItem* child;
    while (!categories_grid_->isEmpty() && (child = categories_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    QFont font_bold;
    font_bold.setBold(true);

    QLabel* cat_label = new QLabel("Category");
    cat_label->setFont(font_bold);
    categories_grid_->addWidget(cat_label, 0, 0);

    QLabel* edition_label = new QLabel("Edition");
    edition_label->setFont(font_bold);
    categories_grid_->addWidget(edition_label, 0, 1);

    if (decode_editable_)
    {
        QLabel* edition_edit_label = new QLabel("Edit");
        edition_edit_label->setFont(font_bold);
        categories_grid_->addWidget(edition_edit_label, 0, 2);
    }

    QLabel* ref_label = new QLabel("REF");
    ref_label->setFont(font_bold);
    categories_grid_->addWidget(ref_label, 0, 3);

    if (decode_editable_)
    {
        QLabel* ref_edit_label = new QLabel("Edit");
        ref_edit_label->setFont(font_bold);
        categories_grid_->addWidget(ref_edit_label, 0, 4);
    }

    QLabel* spf_label = new QLabel("SPF");
    spf_label->setFont(font_bold);
    categories_grid_->addWidget(spf_label, 0, 5);

    if (decode_editable_)
    {
        QLabel* spf_edit_label = new QLabel("Edit");
        spf_edit_label->setFont(font_bold);
        categories_grid_->addWidget(spf_edit_label, 0, 6);
    }

    QIcon edit_icon;
    if (decode_editable_)
        edit_icon = QIcon(Files::IconProvider::getIcon("edit.png"));

    int row = 1;

    category_checkboxes_.clear();
    ref_edit_buttons_.clear();
    spf_edit_buttons_.clear();

    if (ctx_mgr_.hasActiveContext())
    {
        for (auto& cat_it : jasterix_->categories())
        {
            unsigned int category = cat_it.first;
            const std::shared_ptr<Category> cat = cat_it.second;

            logdbg << "cat " << category;

            auto* cfg = ctx_mgr_.asterixConfig(category);
            if (!cfg)
                continue;

            QCheckBox* cat_check = new QCheckBox(String::categoryString(category).c_str());
            cat_check->setObjectName(QString("asterix_cat%1_check").arg(category, 3, 10, QChar('0')));
            cat_check->setProperty("category", category);
            if (decode_editable_)
            {
                cat_check->setChecked(decode_getter_(category));
                connect(cat_check, &QCheckBox::clicked, this, &ASTERIXConfigWidget::categoryCheckedSlot);
            }
            else
            {
                cat_check->setChecked(true);
                cat_check->setEnabled(false);
            }
            categories_grid_->addWidget(cat_check, row, 0);
            category_checkboxes_[category] = cat_check;

            ASTERIXEditionComboBox* ed_combo = new ASTERIXEditionComboBox(cat);
            ed_combo->setObjectName(QString("asterix_cat%1_edition_combo").arg(category, 3, 10, QChar('0')));
            if (cat->editions().count(cfg->edition()))
                ed_combo->setEdition(cfg->edition());
            connect(ed_combo, &ASTERIXEditionComboBox::changedEdition, this,
                    &ASTERIXConfigWidget::editionChangedSlot);
            categories_grid_->addWidget(ed_combo, row, 1);

            if (decode_editable_)
            {
                QPushButton* ed_edit = new QPushButton();
                ed_edit->setObjectName(QString("asterix_cat%1_edition_edit_button").arg(category, 3, 10, QChar('0')));
                ed_edit->setIcon(edit_icon);
                ed_edit->setFixedSize(UI_ICON_SIZE);
                ed_edit->setFlat(UI_ICON_BUTTON_FLAT);
                connect(ed_edit, &QPushButton::clicked, this,
                        &ASTERIXConfigWidget::categoryEditionEditSlot);
                ed_edit->setProperty("category", category);
                categories_grid_->addWidget(ed_edit, row, 2);
            }

            // ref
            ASTERIXREFEditionComboBox* ref_combo = new ASTERIXREFEditionComboBox(cat);
            ref_combo->setObjectName(QString("asterix_cat%1_ref_combo").arg(category, 3, 10, QChar('0')));
            if (cfg->ref().size() && cat->refEditions().count(cfg->ref()))
                ref_combo->setREFEdition(cfg->ref());
            connect(ref_combo, &ASTERIXREFEditionComboBox::changedREFSignal, this,
                    &ASTERIXConfigWidget::refEditionChangedSlot);
            categories_grid_->addWidget(ref_combo, row, 3);

            if (decode_editable_)
            {
                QPushButton* ref_edit = new QPushButton();
                ref_edit->setObjectName(QString("asterix_cat%1_ref_edit_button").arg(category, 3, 10, QChar('0')));
                ref_edit->setIcon(edit_icon);
                ref_edit->setFixedSize(UI_ICON_SIZE);
                ref_edit->setFlat(UI_ICON_BUTTON_FLAT);
                connect(ref_edit, &QPushButton::clicked, this,
                        &ASTERIXConfigWidget::categoryREFEditionEditSlot);
                ref_edit->setProperty("category", category);

                if (!ref_combo->isEnabled())
                    ref_edit->setDisabled(true);

                categories_grid_->addWidget(ref_edit, row, 4);
                ref_edit_buttons_[category] = ref_edit;
            }

            // spf
            ASTERIXSPFEditionComboBox* spf_combo = new ASTERIXSPFEditionComboBox(cat);
            spf_combo->setObjectName(QString("asterix_cat%1_spf_combo").arg(category, 3, 10, QChar('0')));
            if (cfg->spf().size() && cat->spfEditions().count(cfg->spf()))
                spf_combo->setSPFEdition(cfg->spf());
            connect(spf_combo, &ASTERIXSPFEditionComboBox::changedSPFSignal, this,
                    &ASTERIXConfigWidget::spfEditionChangedSlot);
            categories_grid_->addWidget(spf_combo, row, 5);

            if (decode_editable_)
            {
                QPushButton* spf_edit = new QPushButton();
                spf_edit->setObjectName(QString("asterix_cat%1_spf_edit_button").arg(category, 3, 10, QChar('0')));
                spf_edit->setIcon(edit_icon);
                spf_edit->setFixedSize(UI_ICON_SIZE);
                spf_edit->setFlat(UI_ICON_BUTTON_FLAT);
                connect(spf_edit, &QPushButton::clicked, this,
                        &ASTERIXConfigWidget::categorySPFEditionEditSlot);
                spf_edit->setProperty("category", category);

                if (!spf_combo->isEnabled())
                    spf_edit->setDisabled(true);

                categories_grid_->addWidget(spf_edit, row, 6);
                spf_edit_buttons_[category] = spf_edit;
            }

            row++;
        }
    }
}

void ASTERIXConfigWidget::setAllCategoriesDecoded(bool decode)
{
    if (!decode_editable_)
        return;

    loginf << "decode " << decode;

    for (auto& check_it : category_checkboxes_)
    {
        check_it.second->setChecked(decode);
        decode_setter_(check_it.first, decode);
    }

    if (category_checkboxes_.size())
        emit decodingConfigChangedSignal();
}

void ASTERIXConfigWidget::categoryCheckedSlot()
{
    if (!decode_editable_)
        return;

    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);

    QVariant cat_var = widget->property("category");
    bool decode = widget->checkState() == Qt::Checked;
    unsigned int cat = cat_var.toUInt();

    loginf << "cat " << cat;

    decode_setter_(cat, decode);

    emit decodingConfigChangedSignal();
}

void ASTERIXConfigWidget::editionChangedSlot(const std::string& cat_str, const std::string& ed_str)
{
    loginf << "cat " << cat_str << " edition " << ed_str;

    unsigned int cat = std::stoul(cat_str);

    traced_assert(jasterix_->hasCategory(cat));

    ctx_mgr_.setAsterixEdition(cat, ed_str, jasterix_->category(cat)->defaultREFEdition());

    emit decodingConfigChangedSignal();
}

void ASTERIXConfigWidget::refEditionChangedSlot(const std::string& cat_str,
                                                const std::string& ed_str)
{
    loginf << "cat " << cat_str << " ref '" << ed_str << "'";

    unsigned int cat = std::stoul(cat_str);

    traced_assert(jasterix_->hasCategory(cat));

    ctx_mgr_.setAsterixRef(cat, ed_str, jasterix_->category(cat)->defaultEdition());

    traced_assert(!decode_editable_ || ref_edit_buttons_.count(cat));
    if (ref_edit_buttons_.count(cat))
    {
        if (ed_str.size())
            ref_edit_buttons_.at(cat)->setDisabled(false);
        else
            ref_edit_buttons_.at(cat)->setDisabled(true);
    }

    emit decodingConfigChangedSignal();
}

void ASTERIXConfigWidget::spfEditionChangedSlot(const std::string& cat_str,
                                                const std::string& ed_str)
{
    loginf << "cat " << cat_str << " spf '" << ed_str << "'";

    unsigned int cat = std::stoul(cat_str);

    traced_assert(jasterix_->hasCategory(cat));

    ctx_mgr_.setAsterixSpf(cat, ed_str, jasterix_->category(cat)->defaultEdition());

    traced_assert(!decode_editable_ || spf_edit_buttons_.count(cat));
    if (spf_edit_buttons_.count(cat))
    {
        if (ed_str.size())
            spf_edit_buttons_.at(cat)->setDisabled(false);
        else
            spf_edit_buttons_.at(cat)->setDisabled(true);
    }
    
    emit decodingConfigChangedSignal();
}

void ASTERIXConfigWidget::categoryEditionEditSlot()
{
    loginf;

    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);

    QVariant cat_var = widget->property("category");
    unsigned int cat = cat_var.toUInt();
    std::string edition_str;
    if (ctx_mgr_.hasAsterixConfig(cat))
        edition_str = ctx_mgr_.asterixConfig(cat)->edition();
    else
        edition_str = jasterix_->category(cat)->defaultEdition();

    traced_assert(jasterix_->hasCategory(cat));
    traced_assert(jasterix_->category(cat)->hasEdition(edition_str));
    std::string def_path = jasterix_->category(cat)->editionPath(edition_str);

    loginf << "cat " << cat << " path '" << def_path << "'";

    QDesktopServices::openUrl(QUrl(def_path.c_str()));
}

void ASTERIXConfigWidget::categoryREFEditionEditSlot()
{
    loginf;

    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);

    QVariant cat_var = widget->property("category");
    unsigned int cat = cat_var.toUInt();
    std::string ref_edition_str;
    if (ctx_mgr_.hasAsterixConfig(cat))
        ref_edition_str = ctx_mgr_.asterixConfig(cat)->ref();
    else
        ref_edition_str = jasterix_->category(cat)->defaultREFEdition();

    loginf << "ref '" << ref_edition_str << "'";

    traced_assert(jasterix_->hasCategory(cat));
    traced_assert(jasterix_->category(cat)->hasREFEdition(ref_edition_str));
    std::string def_path = jasterix_->category(cat)->refEditionPath(ref_edition_str);

    loginf << "cat " << cat << " ref path '" << def_path << "'";

    QDesktopServices::openUrl(QUrl(def_path.c_str()));
}

void ASTERIXConfigWidget::categorySPFEditionEditSlot()
{
    loginf;

    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);

    QVariant cat_var = widget->property("category");
    unsigned int cat = cat_var.toUInt();
    std::string spf_edition_str;
    if (ctx_mgr_.hasAsterixConfig(cat))
        spf_edition_str = ctx_mgr_.asterixConfig(cat)->spf();
    else
        spf_edition_str = jasterix_->category(cat)->defaultSPFEdition();

    loginf << "spf '" << spf_edition_str << "'";

    traced_assert(jasterix_->hasCategory(cat));
    traced_assert(jasterix_->category(cat)->hasSPFEdition(spf_edition_str));
    std::string def_path = jasterix_->category(cat)->spfEditionPath(spf_edition_str);

    loginf << "cat " << cat << " spf path '" << def_path << "'";

    QDesktopServices::openUrl(QUrl(def_path.c_str()));
}
