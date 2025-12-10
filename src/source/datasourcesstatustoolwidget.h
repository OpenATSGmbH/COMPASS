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

#include "toolboxwidget.h"

class DataSourcesStatusWidget;
class DataSourceManager;

class QComboBox;
class QTextEdit;
class QPushButton;
class QCheckBox;
class QLabel;
class QSplitter;

/**
 */
class DataSourcesStatusToolWidget : public ToolBoxWidget
{
    Q_OBJECT

public:
    typedef std::map<unsigned int, std::vector<unsigned int>> ActiveTrackerLines;

    DataSourcesStatusToolWidget(DataSourceManager& ds_man);
    virtual ~DataSourcesStatusToolWidget();

    //ToolBoxWidget
    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    toolbox::ScreenRatio defaultScreenRatio() const override final;
    void addToConfigMenu(QMenu* menu) override final; 
    void addToToolBar(QToolBar* tool_bar) override final; 
    void loadingStarted() override final;
    void loadingDone() override final;

    void updateContent(bool recreate_required = false);
    void resetStatus();
    void resetEvents();

    static const int DefaultStretchDataSources;
    static const int DefaultStretchEvents;

private:
    void createUI();

    ActiveTrackerLines activeTrackerLines() const;

    void updateTrackerSelection();
    void updateLineSelection(const ActiveTrackerLines& tracker_lines);
    void updateActiveTracker(); 
    void updateEventBox();
    void updateInfos();

    void onDataSourcesChanged();
    void onActiveTrackerChanged();
    
    void addNewestEvent();

    void eventBoxSliderChanged(int value);
    void eventFollowBoxToggled(bool ok);

    DataSourceManager&       ds_man_;
    DataSourcesStatusWidget* ds_widget_{nullptr};

    QSplitter*   main_splitter_          = nullptr;
    QComboBox*   tracker_combo_          = nullptr;
    QComboBox*   line_combo_             = nullptr;
    QLabel*      info_refreshed_label_   = nullptr;
    QLabel*      info_received_label_    = nullptr;
    QTextEdit*   event_box_              = nullptr;
    QPushButton* event_box_reset_button_ = nullptr;

    ActiveTrackerLines current_tracker_lines_;
};
