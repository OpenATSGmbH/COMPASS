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

#include "reconstructortaskdialog.h"
#include "reconstructortask.h"
#include "simplereconstructor.h"
#include "simplereconstructorwidget.h"
#include "global.h"
#include "compass.h"
#include "licensemanager.h"
#include "dbcontentmanager.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "probimmreconstructor.h"
#include "probimmreconstructorwidget.h"
#endif

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QFormLayout>
#include <QComboBox>
#include <QStandardItemModel>
#include <QDesktopServices>
#include <QUrl>

ReconstructorTaskDialog::ReconstructorTaskDialog(ReconstructorTask& task)
    : QDialog(), task_(task)
{
    setWindowTitle("Reconstruct Reference Trajectories");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(1000, 800));

    task_.checkReconstructor();

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* combo_layout = new QFormLayout;
    combo_layout->setMargin(0);
    combo_layout->setFormAlignment(Qt::AlignRight | Qt::AlignTop);

    reconstructor_box_ = new QComboBox();
    reconstructor_box_->addItem(QString::fromStdString(ReconstructorTask::ScoringUMReconstructorName));

    const auto& license_manager = COMPASS::instance().licenseManager();

    bool has_adv_rec = false;

#if USE_EXPERIMENTAL_SOURCE == true
    if (license_manager.componentEnabled(license::License::Component::ComponentProbIMMReconstructor))
    {
        reconstructor_box_->addItem(QString::fromStdString(ReconstructorTask::ProbImmReconstructorName));
        has_adv_rec = true;
    }
#endif

    int idx = reconstructor_box_->findText(QString::fromStdString(task_.currentReconstructorStr()));
    reconstructor_box_->setCurrentIndex(idx);

    connect(reconstructor_box_, &QComboBox::currentTextChanged,
            this, &ReconstructorTaskDialog::reconstructorMethodChangedSlot);

    combo_layout->addRow(tr("Reconstructor Method"), reconstructor_box_);

    reconstructor_info_ = new QLabel;
    reconstructor_info_->setWordWrap(true);
    reconstructor_info_->setVisible(false);
    reconstructor_info_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    auto reconstructor_info2 = new QLabel;
    reconstructor_info2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    QHBoxLayout* info_layout = new QHBoxLayout;
    info_layout->addWidget(reconstructor_info_);
    info_layout->addWidget(reconstructor_info2);

    combo_layout->addRow(tr(""), info_layout);

    main_layout->addLayout(combo_layout);

    reconstructor_widget_stack_ = new QStackedWidget();

    reconstructor_widget_stack_->addWidget(task_.simpleReconstructor()->widget());

#if USE_EXPERIMENTAL_SOURCE == true
    if (license_manager.componentEnabled(license::License::Component::ComponentProbIMMReconstructor))
        reconstructor_widget_stack_->addWidget(task_.probIMMReconstructor()->widget());
#endif

    showCurrentReconstructorWidget();

    main_layout->addWidget(reconstructor_widget_stack_);

    if (!has_adv_rec)
    {
        QString msg = "<b>Please consider supporting us by obtaining a commercial license</b>.<br>This will unlock all advanced reconstructor features. ";
        msg += "<br>For more information contact us at ";
        msg += "<a href='mailto:compass@openats.at'>mailto:compass@openats.at</a>.";

        auto message_label = new QLabel;
        message_label->setWordWrap(true);
        message_label->setText(msg);

        auto cb = [ = ] (const QString& link)
        {
            QDesktopServices::openUrl(QUrl(link));
        };

        connect(message_label, &QLabel::linkActivated, cb);

        main_layout->addWidget(message_label);
    }

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button_);

    button_layout->addStretch();

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(run_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    updateButtons();
    updateReconstructorInfo();
    checkValidity();

    connect (&task_, &ReconstructorTask::configChanged, this, &ReconstructorTaskDialog::checkValidity);
}

ReconstructorTaskDialog::~ReconstructorTaskDialog()
{
}

void ReconstructorTaskDialog::updateReconstructorInfo()
{
    reconstructor_info_->setText("");
    reconstructor_info_->setVisible(false);

    auto rec = task_.currentReconstructor();
    if (!rec)
        return;

    auto info_str = rec->reconstructorInfoString();
    if (!info_str.empty())
    {
        reconstructor_info_->setText(QString::fromStdString(info_str));
        reconstructor_info_->setVisible(true);
    }
}

void ReconstructorTaskDialog::showCurrentReconstructorWidget()
{
    const auto& reconst_str = task_.currentReconstructorStr();
    int idx = reconstructor_box_->findText(QString::fromStdString(reconst_str));

    loginf << "value " << idx;

    traced_assert(idx >= 0);

    reconstructor_widget_stack_->setCurrentIndex(idx);

    task_.currentReconstructor()->updateWidgets();
}

void ReconstructorTaskDialog::updateButtons()
{
    traced_assert(run_button_);

    run_button_->setDisabled(!task_.canRun());
}

void ReconstructorTaskDialog::reconstructorMethodChangedSlot(const QString& value)
{
    loginf << "value " << value.toStdString();

    task_.currentReconstructorStr(value.toStdString());

    showCurrentReconstructorWidget();
    updateReconstructorInfo();
}

void ReconstructorTaskDialog::checkValidity()
{
    auto valid = configValid();

    run_button_->setEnabled(valid.first);
    run_button_->setToolTip(QString::fromStdString(valid.second));
}

std::pair<bool, std::string> ReconstructorTaskDialog::configValid() const
{
    auto rec = task_.currentReconstructor();
    if (!rec)
        return std::make_pair(false, "No reconstructor found");

    //get full data time range
    if (!COMPASS::instance().dbContentManager().hasMinMaxTimestamp())
        return std::make_pair(false, "No timestamps found");

    boost::posix_time::ptime data_t0, data_t1;
    std::tie(data_t0, data_t1) = COMPASS::instance().dbContentManager().minMaxTimestamp();

    const auto& settings = rec->settings();
    if (settings.data_timestamp_min.is_not_a_date_time())
        return std::make_pair(false, "Please enter a valid begin time");
    if (settings.data_timestamp_max.is_not_a_date_time())
        return std::make_pair(false, "Please enter a valid end time");
    if (settings.data_timestamp_max <= settings.data_timestamp_min ||
        settings.data_timestamp_min >= data_t1 ||
        settings.data_timestamp_max <= data_t0)
        return std::make_pair(false, "Please enter a valid timeframe");

    return std::make_pair(true, "");
}
