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

#include "toolbox.h"
#include "toolboxwidget.h"
#include "toolboxwidth.h"

#include "popupmenu.h"

#include "stringconv.h"
#include "files.h"
#include "logger.h"
#include "traced_assert.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QPushButton>
#include <QStackedWidget>
#include <QLabel>
#include <QFontMetrics>
#include <QMenu>
#include <QToolBar>
#include <QSplitter>
#include <QSplitterHandle>
#include <QMouseEvent>
#include <QSettings>

#include <cmath>
#include <functional>

namespace
{
    QString widthSettingsKey(const std::string& tool_name)
    {
        return "WidthPercent/" + QString::fromStdString(tool_name);
    }

    /**
     * Does not move during a drag, it only reports the dragged position. The owner sets the
     * sizes in steps, so the widgets are resized once per step and not once per mouse move.
     */
    class StepSplitterHandle : public QSplitterHandle
    {
    public:
        StepSplitterHandle(QSplitter* parent, const std::function<void(int)>& dragged_cb)
        :   QSplitterHandle(Qt::Horizontal, parent), dragged_cb_(dragged_cb) {}

    protected:
        void mousePressEvent(QMouseEvent* e) override
        {
            press_x_ = e->pos().x();
            QSplitterHandle::mousePressEvent(e);
        }

        void mouseMoveEvent(QMouseEvent* e) override
        {
            if (!(e->buttons() & Qt::LeftButton))
                return;

            dragged_cb_(splitter()->mapFromGlobal(e->globalPos()).x() - press_x_);
        }

    private:
        std::function<void(int)> dragged_cb_;
        int                      press_x_ = 0;
    };

    class StepSplitter : public QSplitter
    {
    public:
        StepSplitter(const std::function<void(int)>& dragged_cb)
        :   QSplitter(Qt::Horizontal), dragged_cb_(dragged_cb) {}

    protected:
        QSplitterHandle* createHandle() override { return new StepSplitterHandle(this, dragged_cb_); }

    private:
        std::function<void(int)> dragged_cb_;
    };
}

const int   ToolBox::ToolIconSize      = 50;
const int   ToolBox::ToolNameFontSize  = 12;
const int   ToolBox::ToolLabelFontSize = 8;
const float ToolBox::ExpansionFactor   = 0.75f;

/**
 */
ToolBox::ToolBox(QWidget* parent)
:   QWidget(parent)
{
    createUI();
}

/**
 */
ToolBox::~ToolBox() = default;

/**
 */
void ToolBox::setMainContent(QWidget* content)
{
    traced_assert(!main_content_widget_);

    main_content_widget_ = content;
    main_content_widget_->setParent(main_widget_);
    main_layout_->addWidget(main_content_widget_);

    main_content_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

/**
 */
void ToolBox::createUI()
{
    QHBoxLayout* layout = new QHBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    setLayout(layout);

    //icons
    QVBoxLayout* icon_bar_layout  = new QVBoxLayout;
    icon_bar_layout->setContentsMargins(0, 0, 0, 0);
    icon_bar_layout->setSpacing(0);

    icon_layout_ = new QVBoxLayout;
    icon_layout_->setContentsMargins(0, 0, 0, 0);
    icon_layout_->setSpacing(0);

    icon_bar_layout->addLayout(icon_layout_);
    icon_bar_layout->addStretch();
    
    layout->addLayout(icon_bar_layout);

    splitter_ = new StepSplitter([ this ] (int panel_width) { this->handleDragged(panel_width); });
    splitter_->setChildrenCollapsible(false);

    layout->addWidget(splitter_);

    //panel
    panel_widget_ = new QWidget;
    panel_widget_->setObjectName("toolbox_panel");
    panel_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    panel_widget_->setVisible(false); //initially invisible

    panel_layout_ = new QVBoxLayout;
    panel_layout_->setContentsMargins(0, 0, 0, 0);
    panel_layout_->setSpacing(0);

    panel_widget_->setLayout(panel_layout_);

    panel_content_widget_ = new QFrame;
    panel_content_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    panel_content_widget_->setFrameStyle(QFrame::Shape::StyledPanel | QFrame::Shadow::Raised);

    QVBoxLayout* panel_content_layout = new QVBoxLayout;
    panel_content_layout->setContentsMargins(0, 0, 0, 0);
    panel_content_layout->setSpacing(0);

    panel_content_widget_->setLayout(panel_content_layout);

    panel_layout_->addWidget(panel_content_widget_);
    
    QHBoxLayout* top_layout = new QHBoxLayout;
    top_layout->setContentsMargins(10, 5, 10, 0);

    tool_name_label_ = new QLabel;
    auto name_font = tool_name_label_->font();
    name_font.setBold(true);
    //name_font.setPointSize(ToolNameFontSize);
    tool_name_label_->setFont(name_font);

    tool_bar_ = new QToolBar;
    tool_bar_->setIconSize(UI_ICON_SIZE);

    config_button_ = new QPushButton;
    config_button_->setObjectName("toolbox_config_button");
    config_button_->setStyleSheet("QPushButton::menu-indicator { image: none; }");
    config_button_->setIcon(Utils::Files::IconProvider::getIcon("edit.png"));
    config_button_->setFixedSize(UI_ICON_SIZE); 
    config_button_->setFlat(UI_ICON_BUTTON_FLAT);

    config_menu_.reset(new PopupMenu(config_button_));
    config_menu_->setPreShowCallback([ = ] () { this->updateMenu(); });

    tool_bar_default_ = new QToolBar;
    tool_bar_default_->setIconSize(UI_ICON_SIZE);

    shrink_action_ = tool_bar_default_->addAction("Decrease Width");
    shrink_action_->setIcon(Utils::Files::IconProvider::getIcon("arrow_to_left.png"));
    shrink_action_->setToolTip("Decrease Width [-]");
    shrink_action_->setShortcut(Qt::Key_Minus);

    connect(shrink_action_, &QAction::triggered, this, &ToolBox::shrink);
    tool_bar_default_->widgetForAction(shrink_action_)->setObjectName("toolbox_shrink_button");

    grow_action_ = tool_bar_default_->addAction("Increase Width");
    grow_action_->setIcon(Utils::Files::IconProvider::getIcon("arrow_to_right.png"));
    grow_action_->setToolTip("Increase Width [+]");
    grow_action_->setShortcut(Qt::Key_Plus);

    connect(grow_action_, &QAction::triggered, this, &ToolBox::grow);
    tool_bar_default_->widgetForAction(grow_action_)->setObjectName("toolbox_grow_button");

    expand_action_ = tool_bar_default_->addAction("Expand");
    expand_action_->setShortcut(Qt::Key_NumberSign);

    connect(expand_action_, &QAction::triggered, this, &ToolBox::toggleExpansion);

    top_layout->addWidget(tool_bar_default_);
    top_layout->addSpacerItem(new QSpacerItem(20, 1, QSizePolicy::Fixed, QSizePolicy::Fixed));
    top_layout->addWidget(tool_name_label_);
    top_layout->addStretch(1);
    top_layout->addWidget(tool_bar_);
    top_layout->addStretch(1);
    top_layout->addWidget(config_button_);

    widget_stack_ = new QStackedWidget;

    panel_content_layout->addLayout(top_layout);
    panel_content_layout->addWidget(widget_stack_);

    splitter_->addWidget(panel_widget_);

    //main content
    main_widget_ = new QWidget;
    main_widget_->setObjectName("toolbox_views");
    main_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    main_layout_ = new QVBoxLayout;
    main_layout_->setContentsMargins(0, 0, 0, 0);
    main_layout_->setSpacing(0);

    main_widget_->setLayout(main_layout_);

    splitter_->addWidget(main_widget_);

    updateToolBar();
    updateButtons();
}

/**
 */
void ToolBox::updateMenu()
{
    config_menu_->clear();

    if (active_tool_idx_ < 0)
        return;

    // tool specific custom part
    tools_.at(active_tool_idx_).widget->addToConfigMenu(config_menu_.get());
}

/**
 */
void ToolBox::updateToolBar()
{
    tool_bar_->clear();

    if (active_tool_idx_ < 0)
        return;

    // tool specific custom part
    tools_.at(active_tool_idx_).widget->addToToolBar(tool_bar_);

    //if (tool_bar_->actions().size() > 0)
    //    tool_bar_->addSeparator();

    //add general actions
}

/**
 */
unsigned int ToolBox::addTool(ToolBoxWidget* tool)
{
    traced_assert(tool);

    auto icon   = tool->toolIcon();
    auto name   = tool->toolName();
    auto info   = tool->toolInfo();
    auto labels = tool->toolLabels();

    std::string label = Utils::String::compress(labels, '\n');

    auto button = new ToolButton;

    auto font = button->font();
    font.setPointSize(ToolLabelFontSize);

    int tool_idx = (int)tools_.size();

    button->setIcon(icon);
    button->setText(QString::fromStdString(label));
    button->setToolTip(QString::fromStdString(info) + " [" + QString::number(tool_idx + 1) + "]");
    button->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextUnderIcon);
    button->setIconSize(QSize(ToolIconSize, ToolIconSize));
    button->setCheckable(true);
    button->setFont(font); 
    button->setShortcut(Qt::Key_1 + tool_idx);

    tool->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    connect(button, &ToolButton::pressed, [ this, tool_idx ] () { this->toolActivated(tool_idx); });
    connect(button, &ToolButton::rightClicked, [ tool ] () { tool->rightClicked(); });
    connect(tool, &ToolBoxWidget::iconChangedSignal, [ button, tool ] { button->setIcon(tool->toolIcon()); } );
    connect(tool, &ToolBoxWidget::toolsChangedSignal, [ this ] () { this->updateToolBar(); });

    unsigned int idx = (unsigned int)tools_.size();

    bool saved_ok = false;
    int  saved    = QSettings("COMPASS", "ToolBox").value(widthSettingsKey(name)).toInt(&saved_ok);

    Tool t;
    t.idx           = tool_idx;
    t.widget        = tool;
    t.button        = button;
    t.width_percent = toolbox::widthPercentOrDefault(saved_ok ? std::optional<int>(saved) : std::nullopt,
                                                     tool->defaultWidthPercent());

    tools_.push_back(t);
    widget_stack_->addWidget(tool);
    icon_layout_->addWidget(button);

    return idx;
}

/**
 */
size_t ToolBox::numTools() const
{
    return tools_.size();
}

void ToolBox::disableTools(std::set<unsigned int> indexes)
{
    // enable all tools

    unsigned int index=0;
    for (const auto& t : tools_)
    {
        bool disable = indexes.count(index);

        if (t.button)
            t.button->setDisabled(disable);

        if (t.widget)
            t.button->setDisabled(disable);

        ++index;
    }
}

/**
 */
void ToolBox::selectTool(size_t idx)
{
    traced_assert(idx >= 0);
    traced_assert(idx < numTools());

    //deactivate any active tool
    toolActivated(-1);

    //activate new tool
    tools_.at(idx).button->blockSignals(true);
    tools_.at(idx).button->setChecked(true);
    tools_.at(idx).button->blockSignals(false);

    toolActivated((int)idx);
}

/**
 */
bool ToolBox::selectTool(const std::string& name)
{
    //find tool of the given name
    auto it = std::find_if(tools_.begin(), tools_.end(), [ & ] (const Tool& t) { return t.widget->toolName() == name; });
    if (it == tools_.end())
        return false;

    //select
    int idx = std::distance(tools_.begin(), it);
    selectTool(idx);

    return true;
}

/**
 */
void ToolBox::toolActivated(int idx)
{
    traced_assert(idx < (int)tools_.size());

    if (idx < 0 && active_tool_idx_ >= 0)
        idx = active_tool_idx_;

    if (idx == active_tool_idx_)
    {
        //close active tool
        panel_widget_->setVisible(false);
        panel_content_widget_->setVisible(false);
        active_tool_idx_ = -1;
    }
    else
    {
        traced_assert(idx >= 0);

        const auto& tool = tools_.at(idx);

        tool_name_label_->setText(QString::fromStdString(tool.widget->toolName()));

        active_tool_idx_ = idx;
        widget_stack_->setCurrentIndex(idx);

        //a stacked widget takes the minimum width of its widest page, so a narrow tool could not be
        //narrower than the widest tool. Only the shown tool counts.
        for (int i = 0; i < widget_stack_->count(); ++i)
        {
            const auto policy = i == idx ? QSizePolicy::Preferred : QSizePolicy::Ignored;
            widget_stack_->widget(i)->setSizePolicy(policy, policy);
        }
        widget_stack_->updateGeometry();

        panel_widget_->setVisible(true);
        panel_content_widget_->setVisible(true);

        for (auto& t : tools_)
        {
            if (t.idx == active_tool_idx_)
                continue;

            t.button->blockSignals(true);
            t.button->setChecked(false);
            t.button->blockSignals(false);
        }
    }

    updateToolBar();
    updateButtons();
    adjustSizings();

    updateMenu();
    config_button_->setVisible(!config_menu_->actions().isEmpty());

    emit toolChanged();
}

/**
 */
void ToolBox::finalize()
{
    size_t maxrows = 0;
    int    maxw    = 0;
    int    maxh    = 0;
    for (const auto& t : tools_)
    {
        auto labels = t.widget->toolLabels();

        if (labels.size() > maxrows)
            maxrows = labels.size();

        for (const auto& l : labels)
        {
            QFontMetrics fm(t.button->font());
            auto r = fm.boundingRect(QString::fromStdString(l));
            if (r.height() > maxh) maxh = r.height();
            if (r.width()  > maxw) maxw = r.width();
        }
    }

    int textw = maxw * 1.1;
    int texth = maxrows * maxh * 1.3;

    int w = std::max(ToolIconSize + 2, textw);
    int h = ToolIconSize + 2 + texth;

    for (auto& t : tools_)
        t.button->setFixedSize(w, h);
}

/**
 */
void ToolBox::adjustSizings()
{
    applyWidthPercent();

    if (expanded_)
    {
        const int x0 = tools_.empty() ? 0 : tools_.at(0).button->width();

        panel_content_widget_->setGeometry(x0, 0, width() * ExpansionFactor, height());
    }
}

/**
 * Sets the splitter to the width of the active tool. QSplitter distributes its width
 * in proportion to the given sizes, so the percent values can be passed directly.
 */
void ToolBox::applyWidthPercent()
{
    if (active_tool_idx_ < 0)
        return;

    const int percent = tools_.at(active_tool_idx_).width_percent;

    const int total = splitter_->width() - splitter_->handleWidth();
    if (total <= 0)
        return;

    //QSplitter does not keep the given ratio if it violates a minimum width, it then falls back to
    //the size hints. So the pixel width is computed here, within the minimum widths of both sides.
    const int panel_min = std::max(panel_widget_->minimumSizeHint().width(), panel_widget_->minimumWidth());
    const int views_min = std::max(main_widget_->minimumSizeHint().width(), main_widget_->minimumWidth());

    int panel_width = (int)std::lround(total * percent / 100.0);
    panel_width = std::min(panel_width, total - views_min);
    panel_width = std::max(panel_width, panel_min);

    splitter_->setSizes({ panel_width, std::max(0, total - panel_width) });
}

/**
 */
void ToolBox::setWidthPercent(int percent)
{
    if (active_tool_idx_ < 0)
        return;

    auto& tool = tools_.at(active_tool_idx_);

    tool.width_percent = toolbox::clampWidthPercent(percent);

    QSettings("COMPASS", "ToolBox").setValue(widthSettingsKey(tool.widget->toolName()), tool.width_percent);

    updateButtons();
}

/**
 * The dragged width snaps to the nearest step. The sizes change only when the step changes.
 */
void ToolBox::handleDragged(int panel_width)
{
    if (expanded_ || active_tool_idx_ < 0)
        return;

    const int total = splitter_->width() - splitter_->handleWidth();
    if (total <= 0)
        return;

    const int percent = toolbox::snapWidthPercent((int)std::lround(100.0 * panel_width / total));

    if (percent == tools_.at(active_tool_idx_).width_percent)
        return;

    setWidthPercent(percent);
    applyWidthPercent();
}

/**
 */
void ToolBox::shrink()
{
    if (expanded_ || active_tool_idx_ < 0)
        return;

    setWidthPercent(toolbox::widthPercentStepDown(tools_.at(active_tool_idx_).width_percent));
    applyWidthPercent();
}

/**
 */
void ToolBox::grow()
{
    if (expanded_ || active_tool_idx_ < 0)
        return;

    setWidthPercent(toolbox::widthPercentStepUp(tools_.at(active_tool_idx_).width_percent));
    applyWidthPercent();
}

/**
 */
void ToolBox::updateButtons()
{
    bool shrink_enabled = true;
    bool grow_enabled   = true;

    if (active_tool_idx_ >= 0)
    {
        const int percent = tools_.at(active_tool_idx_).width_percent;

        shrink_enabled = percent > toolbox::WidthMinPercent;
        grow_enabled   = percent < toolbox::WidthMaxPercent;
    }

    shrink_action_->setEnabled(shrink_enabled && !expanded_);
    grow_action_->setEnabled(grow_enabled && !expanded_);

    expand_action_->setIcon(Utils::Files::IconProvider::getIcon(expanded_ ? "fd_shrink.png" : "fd_expand.png"));
    expand_action_->setToolTip(QString(expanded_ ? "Collapse" : "Expand") + " Flight Deck [#]");
}

/**
 */
void ToolBox::loadingStarted()
{
    // The offline load progress dialog is Qt::ApplicationModal and already
    // blocks the whole UI; in LiveRunning the started→done envelope wraps the
    // entire live session, so disabling the toolbox here would freeze the
    // sidebar + central widget for the duration of live mode. Just propagate.
    for (auto& t : tools_)
        t.widget->loadingStarted();
}

/**
 */
void ToolBox::loadingDone()
{
    for (auto& t : tools_)
        t.widget->loadingDone();
}

/**
 */
void ToolBox::toggleExpansion()
{
    if (active_tool_idx_ < 0)
        return;

    bool is_expanded  = expanded_;
    bool reshow_panel = false;

    if (is_expanded)
    {
        //de-expand
        panel_content_widget_->setAutoFillBackground(false);
        panel_content_widget_->setParent(panel_widget_);
        panel_layout_->addWidget(panel_content_widget_);

        main_widget_->setEnabled(true);
    }
    else
    {
        //expand
        main_widget_->setEnabled(false);

        panel_content_widget_->setVisible(false);
        panel_layout_->removeWidget(panel_content_widget_);
        panel_content_widget_->setParent(this);
        panel_content_widget_->raise();
        panel_content_widget_->setAutoFillBackground(true);

        reshow_panel = true;
    }

    expanded_ = !is_expanded;

    adjustSizings();
    updateButtons();

    //reshow panel after adjusting sizings
    if (reshow_panel)
        panel_content_widget_->show();
}

/**
 */
void ToolBox::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    adjustSizings();
}
