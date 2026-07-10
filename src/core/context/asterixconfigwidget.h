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

#include <jasterix/jasterix.h>

#include <QWidget>
#include <functional>
#include <map>
#include <memory>

namespace context { class DBContextManager; }

class QVBoxLayout;
class QGridLayout;
class QPushButton;
class QCheckBox;

/**
 * Decode getter: returns whether a category should be decoded.
 * Decode setter: sets whether a category should be decoded.
 * When no lambdas are provided, checkboxes are checked and disabled (read-only).
 */
using DecodeGetter = std::function<bool(unsigned int)>;
using DecodeSetter = std::function<void(unsigned int, bool)>;

class ASTERIXConfigWidget : public QWidget
{
    Q_OBJECT

  signals:
    // emitted whenever a setting that affects decoding is changed by the user
    // (category enabled/disabled, edition/REF/SPF changed)
    void decodingConfigChangedSignal();

  public slots:
    void categoryCheckedSlot();
    void editionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void refEditionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void spfEditionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void categoryEditionEditSlot();
    void categoryREFEditionEditSlot();
    void categorySPFEditionEditSlot();

    void updateSlot();

  public:
    ASTERIXConfigWidget(context::DBContextManager& ctx_mgr,
                        DecodeGetter decode_getter = {},
                        DecodeSetter decode_setter = {},
                        QWidget* parent = nullptr);
    virtual ~ASTERIXConfigWidget();

  protected:
    context::DBContextManager& ctx_mgr_;
    std::shared_ptr<jASTERIX::jASTERIX> jasterix_;

    DecodeGetter decode_getter_;
    DecodeSetter decode_setter_;
    bool decode_editable_{false};

    QVBoxLayout* main_layout_{nullptr};
    QGridLayout* categories_grid_{nullptr};

    std::map<unsigned int, QCheckBox*>   category_checkboxes_;
    std::map<unsigned int, QPushButton*> ref_edit_buttons_;
    std::map<unsigned int, QPushButton*> spf_edit_buttons_;

    void initjASTERIX();
    void updateCategories();
    void setAllCategoriesDecoded(bool decode);
};
