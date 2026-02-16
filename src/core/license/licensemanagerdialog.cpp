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

#include "licensemanagerdialog.h"
#include "license.h"
#include "licensewidget.h"
#include "licenseimportdialog.h"
#include "licensemanager.h"

#include "compass.h"
#include "timeconv.h"

#include <QLineEdit>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>

#include <QMessageBox>

/**
*/
LicenseManagerDialog::LicenseManagerDialog(QWidget* parent, 
                                           Qt::WindowFlags f)
:   QDialog(parent, f)
{
    setWindowTitle("Manage Licenses");

    auto layout = new QVBoxLayout;
    setLayout(layout);

    auto layouth = new QHBoxLayout;
    layout->addLayout(layouth);

    auto add_license_button = new QPushButton("Add License");
    remove_license_button_ = new QPushButton("Remove License");

    layouth->addWidget(add_license_button);
    layouth->addWidget(remove_license_button_);
    layouth->addStretch(1);

    QStringList headers;
    headers << "Active";
    headers << "Name";
    headers << "Type";
    headers << "Activation";
    headers << "Expiration";
    headers << "Status";

    idx_id_    = 1;
    state_idx_ = 5;

    license_list_ = new QTreeWidget;
    license_list_->setColumnCount(headers.count());
    license_list_->setHeaderLabels(headers);
    license_list_->setSelectionMode(QTreeWidget::SelectionMode::SingleSelection);

    license_list_->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::ResizeToContents);
    license_list_->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::Stretch);
    license_list_->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::ResizeToContents);
    license_list_->header()->setSectionResizeMode(3, QHeaderView::ResizeMode::ResizeToContents);
    license_list_->header()->setSectionResizeMode(4, QHeaderView::ResizeMode::ResizeToContents);
    license_list_->header()->setSectionResizeMode(5, QHeaderView::ResizeMode::ResizeToContents);

    layout->addWidget(license_list_);

    auto group = new QGroupBox("License Information");
    auto group_layout = new QVBoxLayout;

    group->setLayout(group_layout);

    bool app_image = COMPASS::instance().isAppImage();

    license_widget_ = new LicenseWidget(!app_image);

    group_layout->addWidget(license_widget_);
    layout->addWidget(group);

    auto button_layout = new QHBoxLayout;
    auto close_button  = new QPushButton("Close");

    button_layout->addStretch(1);
    button_layout->addWidget(close_button);

    layout->addStretch(1);
    layout->addLayout(button_layout);

    updateList();

    connect(add_license_button, &QPushButton::pressed, this, &LicenseManagerDialog::addLicense);
    connect(remove_license_button_, &QPushButton::pressed, this, &LicenseManagerDialog::removeCurrentLicense);

    connect(close_button, &QPushButton::pressed, this, &QDialog::reject);

    connect(license_list_, &QTreeWidget::itemSelectionChanged, this, &LicenseManagerDialog::updateLicenseWidget);
}

/**
*/
LicenseManagerDialog::~LicenseManagerDialog()
{
}

/**
*/
void LicenseManagerDialog::selectLicense(const std::string& id)
{
    for (int i = 0; i < license_list_->topLevelItemCount(); ++i)
    {
        auto item = license_list_->topLevelItem(i);
        if (item->isSelected())
            item->setSelected(false);
    }
    for (int i = 0; i < license_list_->topLevelItemCount(); ++i)
    {
        auto item = license_list_->topLevelItem(i);
        if (item->text(idx_id_).toStdString() == id)
            item->setSelected(true);
    }
}

/**
*/
void LicenseManagerDialog::updateList()
{
    license_list_->blockSignals(true);
    license_list_->clear();

    const auto& license_manager = COMPASS::instance().licenseManager();
    auto active_license = license_manager.activeLicense();

    for (const auto& l : license_manager.getLicenses())
    {
        auto item = new QTreeWidgetItem;

        auto lvalid = l.second.validity();
        bool read   = l.second.state == license::License::State::Read;

        QString active_str = active_license && active_license->id == l.first ? QString::fromUtf8("\u2714") : "";
        
        item->setText(0, active_str);
        item->setText(1, QString::fromStdString(l.first));
        item->setText(2, read ? QString::fromStdString(license::License::typeToString(l.second.type)) : "-");
        item->setText(3, read ? QString::fromStdString(Utils::Time::toDateString(l.second.date_activation)) : "-");
        item->setText(4, read ? QString::fromStdString(Utils::Time::toDateString(l.second.date_expiration)) : "-");
        item->setText(5, QString::fromStdString(license::License::stringFromValidity(lvalid.first)));

        QBrush b (QColor(QString::fromStdString(license::License::colorFromValidity(lvalid.first))));
	    item->setForeground(state_idx_, b);

        license_list_->addTopLevelItem(item);
    }

    license_list_->blockSignals(false);

    license_list_->resizeColumnToContents(0);
    license_list_->resizeColumnToContents(2);
    license_list_->resizeColumnToContents(3);
    license_list_->resizeColumnToContents(4);
    license_list_->resizeColumnToContents(5);

    updateLicenseWidget();
}

/**
*/
void LicenseManagerDialog::updateLicenseWidget()
{
    license_widget_->showLicense(nullptr);

    auto items = license_list_->selectedItems();

    remove_license_button_->setEnabled(!items.isEmpty());

    if (items.isEmpty())
        return;

    auto& license_manager = COMPASS::instance().licenseManager();

    auto id = items.front()->text(idx_id_).toStdString();
    const auto& l = license_manager.getLicense(id);

    license_widget_->showLicense(&l);
}

/**
*/
void LicenseManagerDialog::addLicense()
{
    auto& license_manager = COMPASS::instance().licenseManager();
    auto  license_cur     = license_manager.activeLicense();

    bool had_pro_license = license_cur && license_cur->type == license::License::Type::Pro;

    LicenseImportDialog dlg(this);
    dlg.resize(640, 480);

    if (dlg.exec() == QDialog::Rejected)
        return;

    const auto& l = dlg.getLicense();
    if (!l)
    {
        QMessageBox::critical(this, "Error", "Adding new license failed.");
        return;
    }

    if (license_manager.hasLicense(l->id))
    {
        //overwrite existing license?
        QString txt = "License '" + QString::fromStdString(l->id) + "' already exists. Do you want to update the existing license?";
        auto b = QMessageBox::question(this, 
                                       "Update License",
                                       txt,
                                       QMessageBox::StandardButton::Yes,
                                       QMessageBox::StandardButton::No);
        if (b == QMessageBox::StandardButton::No)
            return;

        license_manager.setLicense(*l, false);
    }
    else
    {
        //just add new license
        license_manager.addLicense(*l, false);
    }

    updateList();
    //selectLicense(l->id);

    license_cur = license_manager.activeLicense();
    bool has_pro_license = license_cur && license_cur->type == license::License::Type::Pro;

    if (!had_pro_license && has_pro_license)
    {
        QString msg = "Successfully updated to Pro license. Thank you for supporting us!";
        QMessageBox::information(this, "Pro License", msg);
    }

    if (!license_manager.writeLicenses())
    {
        QMessageBox::critical(this, "Error", "New license imported successfully, but writing licenses failed.");
    }
}

/**
*/
void LicenseManagerDialog::removeCurrentLicense()
{
    auto items = license_list_->selectedItems();
    if (items.isEmpty())
        return;

    auto& license_manager = COMPASS::instance().licenseManager();

    auto id = items.front()->text(idx_id_).toStdString();

    auto b = QMessageBox::question(this, 
                                   "Remove License",
                                   "Do you really want to remove license '" + QString::fromStdString(id) + "'?",
                                   QMessageBox::StandardButton::Yes,
                                   QMessageBox::StandardButton::No);
    if (b == QMessageBox::StandardButton::No)
        return;
    
    license_manager.removeLicense(id, false);

    updateList();

    if (!license_manager.writeLicenses())
    {
        QMessageBox::critical(this, "Error", "License removed, but writing licenses failed.");
    }
}
