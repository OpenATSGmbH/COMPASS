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

#include "viewscreenshotdialog.h"
#include "view.h"
#include "viewwidget.h"
#include "timeconv.h"
#include "compass.h"
#include "files.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QRadioButton>
#include <QToolButton>
#include <QLineEdit>
#include <QPushButton>

#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>

using namespace Utils;

namespace
{
    /**
    */
    class SelectableImage : public QLabel
    {
    public:
        SelectableImage(const QImage& img, 
                        int border = 2,
                        const QColor& border_color = Qt::black,
                        QWidget* parent = nullptr)
        :   QLabel(parent)
        {
            setImage(img, border, border_color);
            setCursor(Qt::PointingHandCursor);
            
            updateCurrentImage();
        }
        virtual ~SelectableImage() = default;

        void setSelectionCallback(const std::function<void()>& selection_cb)
        {
            selection_cb_ = selection_cb;
        }

        void setSelected(bool selected)
        {
            if (selected == selected_)
                return;

            selected_ = selected;

            updateCurrentImage();
        }

    protected:
        void mousePressEvent(QMouseEvent *event) override
        {
            if (event->button() == Qt::LeftButton)
            {
                //select image
                setSelected(true);

                //invoke callback if set
                if (selection_cb_)
                    selection_cb_();
            }
        }

    private:
        void setImage(const QImage& img,
                      int border,
                      const QColor& border_color)
        {
            img_   = img;
            img_b_ = img;

            //generate border variant of the image for selection
            QPainter painter(&img_b_);
            painter.setRenderHint(QPainter::RenderHint::Antialiasing, false);

            int b = std::max(0, border - 1) * 2 + 1;

            QPen pen;
            pen.setColor(border_color);
            pen.setWidth(b);

            painter.setPen(pen);
            painter.drawRect(0, 0, img_.width() - 1, img_.height() - 1);
        }

        void updateCurrentImage()
        {
            setPixmap(QPixmap::fromImage(selected_ ? img_b_ : img_));
        }

        QImage img_;
        QImage img_b_;
        bool   selected_ = false;

        std::function<void()> selection_cb_;
    };
}

/**
*/
ViewScreenshotDialog::ViewScreenshotDialog(View* view, 
                                           QWidget* parent, 
                                           Qt::WindowFlags f) 
:   QDialog(parent, f)
,   view_  (view     )
{
    traced_assert(view_);

    generateScreenshots();
    createUI();

    layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/**
*/
void ViewScreenshotDialog::createUI()
{
    setWindowTitle("Screenshot");

    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    SelectableImage* image_preview_data = new SelectableImage(preview_data_, PreviewBorder, Qt::black);
    SelectableImage* image_preview_view = new SelectableImage(preview_view_, PreviewBorder, Qt::black);

    QGridLayout* layout_previews = new QGridLayout;

    QHBoxLayout* button_data_layout = new QHBoxLayout;
    QHBoxLayout* button_view_layout = new QHBoxLayout;

    button_data_ = new QRadioButton("Data");
    button_view_ = new QRadioButton("View");

    button_data_layout->addStretch(1);
    button_data_layout->addWidget(button_data_);
    button_data_layout->addStretch(1);

    button_view_layout->addStretch(1);
    button_view_layout->addWidget(button_view_);
    button_view_layout->addStretch(1);

    layout_previews->addWidget(image_preview_data, 0, 1);
    layout_previews->addWidget(image_preview_view, 0, 2);
    layout_previews->addLayout(button_data_layout, 1, 1);
    layout_previews->addLayout(button_view_layout, 1, 2);

    layout_previews->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed), 0, 0);
    layout_previews->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed), 0, 3);

    layout->addLayout(layout_previews);

    layout->addStretch(1);

    QHBoxLayout* layout_buttons = new QHBoxLayout;
    layout->addLayout(layout_buttons);

    QPushButton* button_cancel = new QPushButton("Cancel");
    QPushButton* button_clipb  = new QPushButton("Copy to Clipboard");
    QPushButton* button_save   = new QPushButton(QString(" ") + "Save Image as..."); //offset a little from save icon

    button_clipb->setIcon(Utils::Files::IconProvider::getIcon("copy.png"));
    button_save->setIcon(Utils::Files::IconProvider::getIcon("save.png"));

    layout_buttons->addWidget(button_cancel);
    layout_buttons->addStretch(1);
    layout_buttons->addWidget(button_clipb);
    layout_buttons->addWidget(button_save);

    button_data_->setChecked(true);

    auto selection_cb_radio = [ = ] (bool data_selected)
    {
        image_preview_data->setSelected( data_selected);
        image_preview_view->setSelected(!data_selected);
    };
    auto selection_cb_data = [ = ] ()
    {
        button_data_->setChecked(true);
    };
    auto selection_cb_view = [ = ] ()
    {
        button_view_->setChecked(true);
    };

    image_preview_data->setSelectionCallback(selection_cb_data);
    image_preview_view->setSelectionCallback(selection_cb_view);

    selection_cb_radio(true);

    connect(button_data_, &QRadioButton::toggled, selection_cb_radio);

    connect(button_cancel, &QPushButton::pressed, this, &ViewScreenshotDialog::reject         );
    connect(button_clipb , &QPushButton::pressed, this, &ViewScreenshotDialog::copyToClipboard);
    connect(button_save  , &QPushButton::pressed, this, &ViewScreenshotDialog::save           );
}

/**
 */
void ViewScreenshotDialog::generateScreenshots()
{
    traced_assert(view_);

    auto createPreview = [ & ] (QImage& preview,
                                const QImage& img)
    {
        //gray background for preview
        QImage preview_img(PreviewSize, PreviewSize, QImage::Format_RGB32);
        preview_img.fill(Qt::gray);

        //scale down image and keep aspect
        auto scaled_img = img.scaled(PreviewSize, 
                                     PreviewSize, 
                                     Qt::AspectRatioMode::KeepAspectRatio, 
                                     Qt::TransformationMode::SmoothTransformation);

        //center scaled image (aspect)
        int offsx = (PreviewSize - scaled_img.width() ) / 2;
        int offsy = (PreviewSize - scaled_img.height()) / 2;

        QPainter painter(&preview_img);
        painter.drawImage(offsx, offsy, scaled_img);

        preview = preview_img;
    };

    //obtain screenshots
    screener_data_ = view_->renderData();
    screener_view_ = view_->renderView();

    //create preview versions
    createPreview(preview_data_, screener_data_);
    createPreview(preview_view_, screener_view_);
}

/**
 */
void ViewScreenshotDialog::copyToClipboard()
{
    QApplication::clipboard()->setImage(getScreenshot(), QClipboard::Clipboard);
    accept();
}

/**
 */
void ViewScreenshotDialog::save()
{
    auto timestamp = Utils::Time::toString(Utils::Time::currentUTCTime());

    std::string fn_init = view_->classId() + "_" + timestamp + ".png";
    std::string path    = COMPASS::instance().lastUsedPath() + "/" + fn_init;

    QString fn = QFileDialog::getSaveFileName(this, "Select Screenshot File", QString::fromStdString(path), "*.png");
    if (fn.isEmpty())
        return;

    QApplication::setOverrideCursor(Qt::WaitCursor);

    bool ok = getScreenshot().save(fn);

    QApplication::restoreOverrideCursor();

    if (ok)
        COMPASS::instance().lastUsedPath(Files::getDirectoryFromPath(fn.toStdString().c_str()));
    else
        QMessageBox::critical(this, "Error", "Screenshot could not be written.");



    accept();
}

/**
 */
const QImage& ViewScreenshotDialog::getScreenshot() const
{
    return (button_data_->isChecked() ? screener_data_ : screener_view_); 
}
