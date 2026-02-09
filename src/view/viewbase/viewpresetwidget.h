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

#include "viewcomponent.h"
#include "viewpresets.h"
#include "json_fwd.hpp"

#include "ui_test_testable.h"

#include <QWidget>
#include <QDialog>
#include <QImage>

class View;

class QListWidget;
class QToolButton;
class QPushButton;
class QLabel;
class QVBoxLayout;
class QLineEdit;
class QTextEdit;
class QFrame;
class QMenu;
class QDialog;
class QCheckBox;

/**
*/
class ViewPresetEditDialog : public QDialog
{
public:
    enum class Mode
    {   
        Create = 0,
        Edit,
        Copy
    };

    ViewPresetEditDialog(View* view, ViewPresets::Preset* preset, Mode mode, QWidget* parent = nullptr);
    virtual ~ViewPresetEditDialog() = default;

private:
    void createUI();
    void configureUI();

    bool checkName();

    void apply();
    bool applyCreate();
    bool applyEdit();
    bool applyCopy();

    void updatePreview();
    void updateConfig();
    void updateMetaData();

    QString windowTitle(Mode mode) const;
    const QImage& previewImage(Mode mode) const;

    View*                view_   = nullptr;
    ViewPresets::Preset* preset_ = nullptr;

    QLineEdit* name_edit_        = nullptr;
    QLineEdit* category_edit_    = nullptr;
    QTextEdit* description_edit_ = nullptr;
    QLabel*    preview_label_    = nullptr;

    ViewPresets::Preset preset_new_;

    Mode mode_ = Mode::Create;
};

/**
*/
class ViewPresetItemWidget : public QWidget
{
    Q_OBJECT
public:
    ViewPresetItemWidget(const ViewPresets::Key& key,
                         View* view,
                         QWidget* parent = nullptr);
    virtual ~ViewPresetItemWidget() = default;

    const ViewPresets::Preset* getPreset() const { return preset_; }
    const ViewPresets::Key& key() const { return key_; }

    void updateContents(const ViewPresets::Key& key);
    void updateContents();

    void showSeparatorLine(bool show);

    static const int IconHeight = 16;

signals:
    void removePreset(ViewPresets::Key key);
    void editPreset(ViewPresets::Key key);
    void copyPreset(ViewPresets::Key key);
    void presetApplied(ViewPresets::Key key);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void enterEvent(QEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    void createUI();

    void removeButtonPressed();
    void editButtonPressed();
    void copyButtonPressed();
    void saveButtonPressed();
    
    ViewPresets::Key           key_;
    const ViewPresets::Preset* preset_ = nullptr;
    View*                      view_   = nullptr;

    QLabel*      category_label_    = nullptr;
    QLabel*      name_label_        = nullptr;
    QLabel*      deployed_label_    = nullptr;
    QLabel*      preview_label_     = nullptr;
    QLabel*      description_label_ = nullptr;

    QToolButton* remove_button_     = nullptr;
    QToolButton* edit_button_       = nullptr;
    QToolButton* copy_button_       = nullptr;
    QToolButton* save_button_       = nullptr;

    QWidget*     main_widget_       = nullptr;
    QFrame*      separator_line_    = nullptr;

    bool inside_ = false;
};

/**
*/
class ViewPresetItemListWidget : public QWidget
{
    Q_OBJECT
public:
    ViewPresetItemListWidget(View* view,
                             QWidget* parent = nullptr);
    virtual ~ViewPresetItemListWidget() = default;

    void updateContents();

    int count() const;
    std::vector<ViewPresets::Key> keys() const;

    static const double WidgetWFraction;
    static const double WidgetHFraction;

signals:
    void presetApplied(ViewPresets::Key key);

private:
    void createUI();
    void clear();
    void refill();
    void updateFilter();
    void updateMinSize();

    void addPreset();

    void editPreset(ViewPresets::Key key);
    void removePreset(ViewPresets::Key key);
    void copyPreset(ViewPresets::Key key);
    
    void removeItem(ViewPresets::Key key);
    void updateItem(ViewPresets::Key key);

    View* view_ = nullptr;

    QLineEdit*   filter_edit_       = nullptr;
    QCheckBox*   show_deployed_box_ = nullptr;
    QVBoxLayout* item_layout_       = nullptr;

    std::vector<ViewPresetItemWidget*> items_;
};

/**
 * Widget governing a view's presets.
*/
class ViewPresetWidget : public QWidget, public ViewComponent, public ui_test::UITestable
{
public:
    ViewPresetWidget(View* view,
                     QWidget* parent = nullptr);
    virtual ~ViewPresetWidget() = default;

    void updateContents();

    nlohmann::json viewInfoJSON() const override final;

    static const std::string NumPresetsKey;

protected:
    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    boost::optional<QString> uiGet(const QString& what = QString()) const override;
    nlohmann::json uiGetJSON(const QString& what = QString()) const override;
    bool uiSet(const QString& str) override;

private:
    void createUI();
    void presetApplied(ViewPresets::Key key);

    QString generateTooltip() const;
    QString generateButtonText() const;

    View* view_ = nullptr;

    ViewPresetItemListWidget* preset_list_ = nullptr;
    QToolButton*              show_button_ = nullptr;
    QMenu*                    button_menu_ = nullptr;
};
