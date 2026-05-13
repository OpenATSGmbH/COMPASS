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

#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/reportexporter.h"
#include "task/result/taskresult.h"

#include "taskmanager.h"
#include "logger.h"

#include <QResizeEvent>
#include <QString>
#include <QStringList>
#include <QTextDocument>
#include <QTextEdit>
#include <QVBoxLayout>

#include "traced_assert.h"

namespace ResultReport
{

const std::string SectionContentText::FieldTexts = "texts";

/**
 */
SectionContentText::SectionContentText(unsigned int id,
                                       const std::string& name, 
                                       Section* parent_section)
:   SectionContent(ContentType::Text, id, name, parent_section)
{
}

/**
 */
SectionContentText::SectionContentText(Section* parent_section)
:   SectionContent(ContentType::Text, parent_section)
{
}

/**
 */
void SectionContentText::addText(const std::string& text)
{
    texts_.push_back(text);
}

/**
 */
std::string SectionContentText::resourceExtension() const
{
    return ReportExporter::ExportTextFormat;
}

namespace
{
// Read-only text view that sizes to its document height. Used as the
// section's prose container so the report flows naturally with no scroll
// bars and no fixed-height guesswork.
class FittingTextView : public QTextEdit
{
public:
    explicit FittingTextView(QWidget* parent = nullptr) : QTextEdit(parent)
    {
        setReadOnly(true);
        setFrameShape(QFrame::NoFrame);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        document()->setDocumentMargin(0);
        viewport()->setAutoFillBackground(false);
        setStyleSheet("QTextEdit { background: transparent; }");
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    }

    QSize sizeHint() const override
    {
        const int w = viewport()->width() > 0
                          ? viewport()->width()
                          : QTextEdit::sizeHint().width();
        document()->setTextWidth(w);
        return { w, static_cast<int>(document()->size().height()) + 2 };
    }

    QSize minimumSizeHint() const override { return sizeHint(); }

protected:
    void resizeEvent(QResizeEvent* e) override
    {
        QTextEdit::resizeEvent(e);
        document()->setTextWidth(viewport()->width());
        updateGeometry();
    }
};

// Build an HTML rendering of `texts`. Each entry is split on '\n'; a line
// beginning with "- " becomes a list item (consecutive ones grouped into a
// single <ul>), every other non-empty line becomes its own <p>. All line
// content is HTML-escaped so existing plain-prose callers stay safe.
QString textsToHtml(const std::vector<std::string>& texts)
{
    QString html;
    bool in_list = false;

    auto close_list = [&]() {
        if (in_list)
        {
            html += "</ul>";
            in_list = false;
        }
    };

    for (const auto& text : texts)
    {
        QString s = QString::fromStdString(text);
        const QStringList lines = s.split('\n');
        for (const QString& raw : lines)
        {
            const QString line = raw.trimmed();
            if (line.isEmpty())
            {
                close_list();
                continue;
            }
            if (line.startsWith("- "))
            {
                if (!in_list)
                {
                    html += "<ul style='margin-top:4px; margin-bottom:10px;'>";
                    in_list = true;
                }
                html += "<li style='margin-bottom:2px;'>"
                      + line.mid(2).toHtmlEscaped() + "</li>";
            }
            else
            {
                close_list();
                html += "<p style='margin-top:0; margin-bottom:12px;'>"
                      + line.toHtmlEscaped() + "</p>";
            }
        }
        close_list();
    }
    return html;
}
}

/**
 */
void SectionContentText::addContentUI(QVBoxLayout* layout,
                                      bool force_ui_reset)
{
    traced_assert(layout);

    if (isLocked())
    {
        layout->addWidget(lockStatePlaceholderWidget());
        return;
    }

    if (texts_.empty())
        return;

    auto* view = new FittingTextView;
    view->setHtml(textsToHtml(texts_));
    layout->addWidget(view);
}

/**
 */
const std::vector<std::string>& SectionContentText::texts() const
{
    return texts_;
}

/**
 */
void SectionContentText::clearContent_impl()
{
    texts_.clear();
}

/**
 */
void SectionContentText::toJSON_impl(nlohmann::json& j) const
{
    //call base
    SectionContent::toJSON_impl(j);

    j[ FieldTexts ] = texts_;
}

/**
 */
bool SectionContentText::fromJSON_impl(const nlohmann::json& j)
{
    //call base
    if (!SectionContent::fromJSON_impl(j))
        return false;
    
    if (!j.is_object() ||
        !j.contains(FieldTexts))
    {
        logerr << "section content text does not obtain needed fields";
        return false;
    }

    texts_ = j[ FieldTexts ].get<std::vector<std::string>>();

    return true;
}

/**
 */
Result SectionContentText::toJSONDocument_impl(nlohmann::json& j, 
                                               const std::string* resource_dir,
                                               ReportExportMode export_style) const
{
    //call base
    auto r = SectionContent::toJSONDocument_impl(j, resource_dir, export_style);
    if (!r.ok())
        return r;

    j[ FieldTexts ] = texts_;

    return Result::succeeded();
}

}
