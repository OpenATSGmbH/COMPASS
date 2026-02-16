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

#include <string>
#include <vector>
#include <functional>

#include <QString>
#include <QKeySequence>

class QObject;
class QWidget;

/**
 */
namespace ui_test
{
    /**
     * How to deal with page breaks (\n, \r) in injected key strings.
     */
    enum class PageBreakMode
    {
        Forbidden = 0, //fail if a page break is encountered
        Remove,        //remove page breaks
        SplitText      //split text at page breaks and inject page breaks as enter keys
    };

    //basic ui trigger events
    bool injectKeyEvent(QWidget* root,
                        const QString& obj_name, 
                        const Qt::Key& key, 
                        int delay = -1);
    bool injectKeysEvent(QWidget* root,
                         const QString& obj_name, 
                         const QString& keys, 
                         PageBreakMode pb_mode,
                         int delay = -1);
    bool injectKeyCmdEvent(QWidget* root,
                           const QString& obj_name,
                           const Qt::Key& key,
                           Qt::KeyboardModifiers modifier,
                           int delay = -1);
    bool injectKeySequenceEvent(QWidget* root,
                                const QString& obj_name,
                                const QKeySequence& keys,
                                int delay = -1);
    bool injectClickEvent(QWidget* root,
                          const QString& obj_name, 
                          int x = -1, 
                          int y = -1, 
                          Qt::MouseButton button = Qt::LeftButton, 
                          int delay = -1);
    bool injectRectEvent(QWidget* root,
                         const QString& obj_name, 
                         int x0 = -1, 
                         int y0 = -1, 
                         int x1 = -1, 
                         int y1 = -1,
                         Qt::MouseButton button = Qt::LeftButton, 
                         int delay = -1);

    typedef std::function<bool()> EventFunc;

    bool injectPostModalEvent(QWidget* root,
                              const QString& obj_name,
                              const EventFunc& modal_trigger_cb,
                              const EventFunc& post_modal_cb);

    //special ui trigger events
    bool injectMenuBarEvent(QWidget* root,
                            const QString& obj_name,
                            const QStringList& path_to_action,
                            int delay = -1);
    bool injectMenuEvent(QWidget* root,
                         const QString& obj_name,
                         const QStringList& path_to_action,
                         int delay = -1);
    bool injectPopupMenuEvent(const QStringList& path_to_action,
                              int delay = -1);
    bool injectComboBoxEditEvent(QWidget* root,
                                 const QString& obj_name,
                                 const QString& entry_txt,
                                 int delay = -1);
    bool injectComboBoxEditEvent(QWidget* root,
                                 const QString& obj_name,
                                 int entry_idx,
                                 int delay = -1);
    bool injectTabSelectionEvent(QWidget* root,
                                 const QString& obj_name,
                                 const QString& tab_txt,
                                 int delay = -1);
    bool injectToolSelectionEvent(QWidget* root,
                                  const QString& obj_name,
                                  const QString& tool_name,
                                  int delay = -1);
    bool injectLineEditEvent(QWidget* root,
                             const QString& obj_name,
                             const QString& text,
                             int delay = -1);
    bool injectTextEditEvent(QWidget* root,
                             const QString& obj_name,
                             const QString& text,
                             int delay = -1);
    bool injectSpinBoxEvent(QWidget* root,
                            const QString& obj_name,
                            int value,
                            int delay = -1);
    bool injectDoubleSpinBoxEvent(QWidget* root,
                                  const QString& obj_name,
                                  double value,
                                  int delay = -1);
    bool injectSliderEditEvent(QWidget* root,
                               const QString& obj_name,
                               double percent,
                               int delay = -1);
    bool injectScrollEditEvent(QWidget* root,
                               const QString& obj_name,
                               double percent,
                               int delay = -1);
    bool injectCheckBoxEvent(QWidget* root,
                             const QString& obj_name,
                             bool on,
                             int delay = -1);
    bool injectButtonMenuEvent(QWidget* root,
                               const QString& obj_name,
                               const QStringList& path_to_action,
                               int x = -1,
                               int y = -1,
                               int delay = -1);
    bool injectDialogEvent(QWidget* root,
                           const QString& obj_name,
                           bool accept,
                           int delay = -1);
    bool injectWidgetEvent(QWidget* root,
                           const QString& obj_name,
                           const QString& slot_name,
                           int delay = -1);
} // namespace ui_test
