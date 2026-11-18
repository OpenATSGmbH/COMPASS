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
#include "deckbutton.h"
#include "viewdeck.h"

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

const int ToolBox::ToolNameFontSize = 12;

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
 * Takes the view area of the View Deck into the splitter. The icon bar of the View Deck is placed by the
 * main window. The View Deck asks to collapse or to open, the tool box decides and tells the result.
 */
void ToolBox::setViewDeck(ViewDeck* view_deck)
{
    traced_assert(view_deck);
    traced_assert(!main_content_widget_);

    main_content_widget_ = view_deck->viewArea();
    main_content_widget_->setParent(main_widget_);
    main_layout_->addWidget(main_content_widget_);

    main_content_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(view_deck, &ViewDeck::collapseRequestedSignal, this, &ToolBox::collapseViewDeckSlot);
    connect(view_deck, &ViewDeck::openRequestedSignal, this, &ToolBox::openViewDeckSlot);
    connect(this, &ToolBox::viewDeckCollapsedSignal, view_deck, &ViewDeck::setCollapsedSlot);
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

    if (activeTool() < 0)
        return;

    // tool specific custom part
    tools_.at(activeTool()).widget->addToConfigMenu(config_menu_.get());
}

/**
 */
void ToolBox::updateToolBar()
{
    tool_bar_->clear();

    if (activeTool() < 0)
        return;

    // tool specific custom part
    tools_.at(activeTool()).widget->addToToolBar(tool_bar_);

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

    auto button = new DeckButton;

    int tool_idx = (int)tools_.size();

    button->setIcon(icon);
    button->setText(QString::fromStdString(label));
    button->setToolTip(QString::fromStdString(info) + " [" + QString::number(tool_idx + 1) + "]");
    button->setShortcut(Qt::Key_1 + tool_idx);

    tool->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(button, &DeckButton::pressed, [ this, tool_idx ] () { this->toolActivated(tool_idx); });
    connect(button, &DeckButton::rightClickedSignal, [ tool ] () { tool->rightClicked(); });
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
    traced_assert(idx < numTools());

    setState(deck::opened(deck_layout_, deck::Deck::Flight), (int)idx);
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
 * A press on a tool button: the active tool collapses the Flight Deck, any other tool is shown.
 */
void ToolBox::toolActivated(int idx)
{
    traced_assert(idx >= 0 && idx < (int)tools_.size());

    if (idx == activeTool())
        setState(deck::collapsed(deck_layout_, deck::Deck::Flight), tool_idx_);
    else
        setState(deck::opened(deck_layout_, deck::Deck::Flight), idx);
}

/**
 * A click on the selected view. The Flight Deck opens with its last tool if it is collapsed.
 */
void ToolBox::collapseViewDeckSlot()
{
    setState(deck::collapsed(deck_layout_, deck::Deck::View), tool_idx_);
}

/**
 */
void ToolBox::openViewDeckSlot()
{
    setState(deck::opened(deck_layout_, deck::Deck::View), tool_idx_);
}

/**
 * The tool shown in the tool panel, -1 if the Flight Deck is collapsed.
 */
int ToolBox::activeTool() const
{
    return deck::isOpen(deck_layout_, deck::Deck::Flight) ? tool_idx_ : -1;
}

/**
 */
bool ToolBox::viewDeckOpen() const
{
    return deck::isOpen(deck_layout_, deck::Deck::View);
}

/**
 * The only place that changes the open decks and the selected tool, and that shows them.
 */
void ToolBox::setState(deck::Layout layout, int tool_idx)
{
    traced_assert(tool_idx < (int)tools_.size());
    traced_assert(tool_idx >= 0 || !deck::isOpen(layout, deck::Deck::Flight));

    const int  tool_before           = activeTool();
    const bool view_deck_open_before = viewDeckOpen();

    deck_layout_ = layout;
    tool_idx_    = tool_idx;

    const int active_tool = activeTool();

    if (active_tool >= 0)
    {
        tool_name_label_->setText(QString::fromStdString(tools_.at(active_tool).widget->toolName()));

        widget_stack_->setCurrentIndex(active_tool);

        //a stacked widget takes the minimum width of its widest page, so a narrow tool could not be
        //narrower than the widest tool. Only the shown tool counts.
        for (int i = 0; i < widget_stack_->count(); ++i)
        {
            const auto policy = i == active_tool ? QSizePolicy::Preferred : QSizePolicy::Ignored;
            widget_stack_->widget(i)->setSizePolicy(policy, policy);
        }
        widget_stack_->updateGeometry();
    }

    panel_widget_->setVisible(active_tool >= 0);
    main_widget_->setVisible(viewDeckOpen());

    for (auto& t : tools_)
        t.button->setChecked(t.idx == active_tool);

    updateToolBar();
    updateButtons();
    adjustSizings();

    updateMenu();
    config_button_->setVisible(!config_menu_->actions().isEmpty());

    if (viewDeckOpen() != view_deck_open_before)
        emit viewDeckCollapsedSignal(!viewDeckOpen());

    if (active_tool != tool_before)
        emit toolChanged();
}

/**
 */
void ToolBox::finalize()
{
    if (tools_.empty())
        return;

    std::vector<std::vector<std::string>> labels;
    for (const auto& t : tools_)
        labels.push_back(t.widget->toolLabels());

    const QSize size = DeckButton::sizeForLabels(labels, tools_.front().button->font());

    for (auto& t : tools_)
        t.button->setFixedSize(size);
}

/**
 */
void ToolBox::adjustSizings()
{
    applyWidthPercent();
}

/**
 * Sets the splitter to the width of the active tool. QSplitter distributes its width
 * in proportion to the given sizes, so the percent values can be passed directly.
 */
void ToolBox::applyWidthPercent()
{
    //with the View Deck collapsed the tool panel is alone in the splitter and takes its width
    if (activeTool() < 0 || !viewDeckOpen())
        return;

    const int percent = tools_.at(activeTool()).width_percent;

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
    if (activeTool() < 0)
        return;

    auto& tool = tools_.at(activeTool());

    tool.width_percent = toolbox::clampWidthPercent(percent);

    QSettings("COMPASS", "ToolBox").setValue(widthSettingsKey(tool.widget->toolName()), tool.width_percent);

    updateButtons();
}

/**
 * The dragged width snaps to the nearest step. The sizes change only when the step changes.
 */
void ToolBox::handleDragged(int panel_width)
{
    if (activeTool() < 0 || !viewDeckOpen())
        return;

    const int total = splitter_->width() - splitter_->handleWidth();
    if (total <= 0)
        return;

    const int percent = toolbox::snapWidthPercent((int)std::lround(100.0 * panel_width / total));

    if (percent == tools_.at(activeTool()).width_percent)
        return;

    setWidthPercent(percent);
    applyWidthPercent();
}

/**
 */
void ToolBox::shrink()
{
    if (activeTool() < 0 || !viewDeckOpen())
        return;

    setWidthPercent(toolbox::widthPercentStepDown(tools_.at(activeTool()).width_percent));
    applyWidthPercent();
}

/**
 */
void ToolBox::grow()
{
    if (activeTool() < 0 || !viewDeckOpen())
        return;

    setWidthPercent(toolbox::widthPercentStepUp(tools_.at(activeTool()).width_percent));
    applyWidthPercent();
}

/**
 */
void ToolBox::updateButtons()
{
    bool shrink_enabled = true;
    bool grow_enabled   = true;

    if (activeTool() >= 0)
    {
        const int percent = tools_.at(activeTool()).width_percent;

        shrink_enabled = percent > toolbox::WidthMinPercent;
        grow_enabled   = percent < toolbox::WidthMaxPercent;
    }

    //no width to set while the tool panel is alone in the splitter
    shrink_action_->setEnabled(shrink_enabled && viewDeckOpen());
    grow_action_->setEnabled(grow_enabled && viewDeckOpen());
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
void ToolBox::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    adjustSizings();
}
