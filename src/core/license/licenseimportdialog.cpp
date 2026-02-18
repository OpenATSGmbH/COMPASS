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

#include "licenseimportdialog.h"
#include "license.h"
#include "licensewidget.h"

#include "compass.h"
#include "timeconv.h"
#include "files.h"

#include <fstream>

#include <QLineEdit>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>

/**
*/
class KeyEdit : public QTextEdit
{
public:
    KeyEdit(QWidget* parent = nullptr) : QTextEdit(parent) {}
    virtual ~KeyEdit() = default;
protected:
    virtual void dragEnterEvent(QDragEnterEvent *event) override
    {
        if (event->mimeData()->hasUrls())
        {
            auto urls = event->mimeData()->urls();
            if (!urls.empty())
            {
                QString fn = urls[ 0 ].toLocalFile();
                if (fn.endsWith(".txt"))
                    event->acceptProposedAction();
            }
        }
    }
    virtual void dropEvent(QDropEvent *event) override
    {
        std::string drop_fn;

        if (event->mimeData()->hasUrls())
        {
            auto urls = event->mimeData()->urls();
            if (!urls.empty())
            {
                QString fn = urls[ 0 ].toLocalFile();
                if (fn.endsWith(".txt"))
                    drop_fn = fn.toStdString();
            }
        }

        if (drop_fn.empty() || !Utils::Files::fileExists(drop_fn))
            return;

        if (readFile(drop_fn))
            event->acceptProposedAction();
    }

private:
    bool readFile(const std::string& fn)
    {
        std::ifstream file(fn);
        if (!file.is_open())
            return false;

        std::string str, file_contents;
        while (std::getline(file, str))
            file_contents += str;
        
        setText(QString::fromStdString(file_contents));

        return !file_contents.empty();
    }
};

/**
*/
LicenseImportDialog::LicenseImportDialog(QWidget* parent, 
                                         Qt::WindowFlags f)
:   QDialog(parent, f)
{
    setWindowTitle("Add New License");

    auto layout = new QVBoxLayout;
    setLayout(layout);

    auto key_layout = new QFormLayout;
    auto key_edit   = new KeyEdit;

    key_edit->setPlaceholderText("Enter license key");
    key_edit->setAcceptRichText(false);
    key_edit->setAcceptDrops(true);
    
    key_layout->addRow("License Key: ", key_edit);

    status_label_ = new QLabel;

    key_layout->addRow("", status_label_);

    layout->addLayout(key_layout);

    auto group = new QGroupBox("License Information");
    auto group_layout = new QVBoxLayout;

    group->setLayout(group_layout);

    bool app_image = COMPASS::instance().isAppImage();

    license_widget_ = new LicenseWidget(!app_image);
    group_layout->addWidget(license_widget_);

    layout->addWidget(group);

    layout->addStretch(1);

    auto button_layout = new QHBoxLayout;
    import_button_     = new QPushButton("Add License");
    auto button_cancel = new QPushButton("Cancel");

    button_layout->addWidget(button_cancel);
    button_layout->addStretch(1);
    button_layout->addWidget(import_button_);

    layout->addLayout(button_layout);

    importLicenseKey("");

    connect(key_edit, &QTextEdit::textChanged, [ = ] () { this->importLicenseKey(key_edit->toPlainText()); });
    connect(import_button_, &QPushButton::pressed, this, &QDialog::accept);
    connect(button_cancel, &QPushButton::pressed, this, &QDialog::reject);
}

/**
*/
LicenseImportDialog::~LicenseImportDialog()
{
}

/**
*/
void LicenseImportDialog::importLicenseKey(const QString& license_key)
{
    license_.reset();

    license::License l;

    status_label_->setText("");
    status_label_->setStyleSheet("");
    license_widget_->showLicense(nullptr);
    import_button_->setEnabled(false);

    if (license_key.isEmpty())
        return;

    //read license
    l.read(license_key.toStdString());

    //show license
    license_widget_->showLicense(&l);

    //verify license
    auto validity = l.validity();
    
    status_label_->setText("License " + QString::fromStdString(license::License::stringFromValidity(validity.first)).toLower());
    status_label_->setStyleSheet("QLabel { color: " + QString::fromStdString(license::License::colorFromValidity(validity.first)) + "; }");

    import_button_->setEnabled(validity.first == license::License::Validity::Valid ||
                               validity.first == license::License::Validity::NotActivated);

    license_.reset(new license::License);
    *license_ = l;
}
