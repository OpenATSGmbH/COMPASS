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

#include "viewdeck.h"

#include "deckbutton.h"
#include "files.h"
#include "global.h"
#include "traced_assert.h"
#include "ui_test_common.h"

#include <QEvent>
#include <QPushButton>
#include <QStackedWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>

namespace
{
    /**
     * Close icon in the top right corner of a deck button. It is painted only while the mouse is over
     * the deck button. It takes clicks at any time, so the UI tests reach it without a mouse move.
     */
    class HoverCloseButton : public QToolButton
    {
    public:
        HoverCloseButton(QWidget* deck_button)
        :   QToolButton(deck_button)
        {
            setIcon(Utils::Files::IconProvider::getIcon("delete.png"));
            setIconSize(QSize(Size - 6, Size - 6));
            setFixedSize(Size, Size);
            setAutoRaise(true);

            deck_button->installEventFilter(this);
        }

        static const int Size = 16;

    protected:
        bool eventFilter(QObject* obj, QEvent* e) override
        {
            if (e->type() == QEvent::Enter || e->type() == QEvent::Leave)
                update();

            return QToolButton::eventFilter(obj, e);
        }

        void paintEvent(QPaintEvent* e) override
        {
            if (parentWidget()->underMouse())
                QToolButton::paintEvent(e);
        }
    };
}

/**
 */
ViewDeck::ViewDeck(QWidget* parent)
:   QWidget(parent)
{
    setObjectName("view_deck");

    view_area_ = new QStackedWidget;

    connect(view_area_.data(), &QStackedWidget::widgetRemoved, this, &ViewDeck::viewRemoved);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    setLayout(layout);

    button_layout_ = new QVBoxLayout;
    button_layout_->setContentsMargins(0, 0, 0, 0);
    button_layout_->setSpacing(0);

    add_button_ = new QPushButton;
    add_button_->setObjectName("add_view");
    add_button_->setIcon(Utils::Files::IconProvider::getIcon("crosshair_fat.png"));
    add_button_->setFixedSize(UI_ICON_SIZE);
    add_button_->setFlat(UI_ICON_BUTTON_FLAT);
    add_button_->setToolTip("Add View");

    connect(add_button_, &QPushButton::clicked, this, &ViewDeck::addViewRequestedSignal);

    layout->addLayout(button_layout_);
    layout->addSpacing(5);
    layout->addWidget(add_button_, 0, Qt::AlignHCenter);
    layout->addStretch();

    // the icon bar keeps its width without views
    setMinimumWidth(DeckButton::sizeForLabels({}, font()).width());
}

/**
 */
ViewDeck::~ViewDeck()
{
    if (view_area_)
    {
        view_area_->disconnect(this);
        delete view_area_;
    }
}

/**
 */
QWidget* ViewDeck::viewArea() const
{
    return view_area_;
}

/**
 */
void ViewDeck::addView(QWidget* view_widget,
                       const std::string& instance_name,
                       const QString& name,
                       const QString& label,
                       const QIcon& icon,
                       const QString& tool_tip_info)
{
    traced_assert(view_widget);
    traced_assert(indexOf(view_widget) < 0);

    const QString object_name = "view_deck_" + ui_test::normalizedObjectName(QString::fromStdString(instance_name));

    auto button = new DeckButton;
    button->setObjectName(object_name);
    button->setIcon(icon);
    button->setText(label);
    button->setToolTip(tool_tip_info.isEmpty() ? name : name + "\n" + tool_tip_info);

    auto close_button = new HoverCloseButton(button);
    close_button->setObjectName(object_name + "_close");
    close_button->setToolTip("Close View");
    close_button->setEnabled(add_remove_enabled_);

    connect(button, &DeckButton::pressed, this, [ this, view_widget ] () { this->buttonPressed(view_widget); });
    connect(close_button, &QToolButton::clicked,
            this, [ this, view_widget ] () { emit closeViewRequestedSignal(view_widget); });

    Entry entry;
    entry.widget       = view_widget;
    entry.button       = button;
    entry.close_button = close_button;
    entry.name         = name;

    entries_.push_back(entry);
    button_layout_->addWidget(button);

    // Suppress paint events during the insertion. QStackedWidget::addWidget triggers
    // QWidget::setParent which can fire a paint event on the widget being
    // reparented. For views containing a QOpenGLWidget (Geographic View) this
    // causes OSG to render into a stale GL context and crash in SceneView::cull.
    view_widget->setUpdatesEnabled(false);
    view_area_->addWidget(view_widget);
    view_widget->setUpdatesEnabled(true);

    updateButtonSizes();
    updateSizePolicies();
    updateButtons();
}

/**
 */
void ViewDeck::setViewEnabled(QWidget* view_widget, bool enabled)
{
    const int index = indexOf(view_widget);
    traced_assert(index >= 0);

    entries_[ index ].enabled = enabled;
    entries_[ index ].button->setEnabled(enabled);
    view_widget->setEnabled(enabled);

    if (enabled || view_widget != currentView())
        return;

    // a disabled view does not stay selected: the next enabled view takes over, else the first enabled one
    auto is_enabled = [] (const Entry& e) { return e.enabled; };

    auto it = std::find_if(entries_.begin() + index, entries_.end(), is_enabled);
    if (it == entries_.end())
        it = std::find_if(entries_.begin(), entries_.end(), is_enabled);

    if (it != entries_.end())
        setCurrentView(it->widget);
}

/**
 */
void ViewDeck::showView(QWidget* view_widget)
{
    traced_assert(indexOf(view_widget) >= 0);

    setCurrentView(view_widget);

    if (collapsed_)
        emit openRequestedSignal();
}

/**
 */
QWidget* ViewDeck::currentView() const
{
    return view_area_->currentWidget();
}

/**
 */
void ViewDeck::setAddRemoveEnabled(bool enabled)
{
    add_remove_enabled_ = enabled;

    add_button_->setEnabled(enabled);

    for (auto& e : entries_)
        e.close_button->setEnabled(enabled);
}

/**
 */
bool ViewDeck::uiSet(const QString& str)
{
    auto it = std::find_if(entries_.begin(), entries_.end(), [ & ] (const Entry& e) { return e.name == str; });
    if (it == entries_.end())
        return false;

    showView(it->widget);

    return true;
}

/**
 */
void ViewDeck::setCollapsedSlot(bool collapsed)
{
    collapsed_ = collapsed;

    updateButtons();
}

/**
 */
int ViewDeck::indexOf(QWidget* view_widget) const
{
    auto it = std::find_if(entries_.begin(), entries_.end(),
                           [ & ] (const Entry& e) { return e.widget == view_widget; });

    return it == entries_.end() ? -1 : (int)std::distance(entries_.begin(), it);
}

/**
 */
void ViewDeck::setCurrentView(QWidget* view_widget)
{
    view_area_->setCurrentWidget(view_widget);

    updateSizePolicies();
    updateButtons();
}

/**
 * A press on the selected view asks the host to collapse the deck, any other press shows the view.
 */
void ViewDeck::buttonPressed(QWidget* view_widget)
{
    if (!collapsed_ && view_widget == currentView())
        emit collapseRequestedSignal();
    else
        showView(view_widget);
}

/**
 * Called when a view widget left the view area, which also happens when the view is deleted.
 * The view area has selected the neighbor view at this point.
 */
void ViewDeck::viewRemoved(int index)
{
    traced_assert(index >= 0 && index < (int)entries_.size());

    auto button = entries_[ index ].button;

    entries_.erase(entries_.begin() + index);

    // not deleted directly: its close button can be the sender of the close request
    button_layout_->removeWidget(button);
    button->hide();
    button->deleteLater();

    updateButtonSizes();
    updateSizePolicies();
    updateButtons();
}

/**
 */
void ViewDeck::updateButtons()
{
    const QWidget* current = currentView();

    for (auto& e : entries_)
        e.button->setChecked(!collapsed_ && e.widget == current);
}

/**
 */
void ViewDeck::updateButtonSizes()
{
    if (entries_.empty())
        return;

    std::vector<std::vector<std::string>> labels;
    for (const auto& e : entries_)
        labels.push_back({ e.button->text().toStdString() });

    const QSize size = DeckButton::sizeForLabels(labels, entries_.front().button->font());

    for (auto& e : entries_)
    {
        e.button->setFixedSize(size);
        e.close_button->move(size.width() - e.close_button->width() - 1, 1);
    }
}

/**
 * A stacked widget takes the minimum size of its largest page, so a narrow view could not be narrower than the
 * widest view of the deck. Only the shown view counts.
 */
void ViewDeck::updateSizePolicies()
{
    const QWidget* current = currentView();

    for (auto& e : entries_)
    {
        const auto policy = e.widget == current ? QSizePolicy::Preferred : QSizePolicy::Ignored;
        e.widget->setSizePolicy(policy, policy);
    }

    view_area_->updateGeometry();
}
