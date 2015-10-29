/*
 * bm_tuning_panel.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/tuning_panel_bm.hpp>
#include <QString>
#include <QApplication>

namespace reco {
namespace stereo_workbench {

tuning_panel_bm::tuning_panel_bm(QWidget* parent):tuning_panel(parent){
	construct_specialized_controls();
}

void tuning_panel_bm::construct_specialized_controls(){
	//************************************** pre-filter cap ****************************************
	construct_integer_control_set(
		pre_filter_cap_horizontal_layout,
		pre_filter_cap_label,
		pre_filter_cap_spin_box,
		pre_filter_cap_slider,
		tr("pre-filter cap:"),
		QStringLiteral("pre_filter_cap_horizontal_layout"),
		QStringLiteral("pre_filter_cap_label"),
		QStringLiteral("pre_filter_cap_spin_box"),
		QStringLiteral("pre_filter_cap_slider"),
		0,256
		);

	construct_integer_control_set(
		pre_filter_size_horizontal_layout,
		pre_filter_size_label,
		pre_filter_size_spin_box,
		pre_filter_size_slider,
		tr("pre-filter size:"),
		QStringLiteral("pre_filter_size_horizontal_layout"),
		QStringLiteral("pre_filter_size_label"),
		QStringLiteral("pre_filter_size_spin_box"),
		QStringLiteral("pre_filter_size_slider"),
		0,256
		);


	pre_filter_type_combo_box = new QComboBox(this);
	pre_filter_type_combo_box->setObjectName(QStringLiteral("pre_filter_type_combo_box"));
	tuning_controls_vlayout->addWidget(pre_filter_type_combo_box);
	pre_filter_type_combo_box->clear();
	pre_filter_type_combo_box->insertItems(0, QStringList()
	 << QApplication::translate("main_window", "normalized response", 0)
	 << QApplication::translate("main_window", "x-sobel", 0)
	);

	construct_integer_control_set(
		smaller_block_size_horizontal_layout,
		smaller_block_size_label,
		smaller_block_size_spin_box,
		smaller_block_size_slider,
		tr("smaller block size:"),
		QStringLiteral("smaller_block_size_horizontal_layout"),
		QStringLiteral("smaller_block_size_label"),
		QStringLiteral("smaller_block_size_spin_box"),
		QStringLiteral("smaller_block_size_slider"),
		1,11,2,1
		);

	construct_integer_control_set(
		texture_threshold_horizontal_layout,
		texture_threshold_label,
		texture_threshold_spin_box,
		texture_threshold_slider,
		tr("texture threshold:"),
		QStringLiteral("texture_threshold_horizontal_layout"),
		QStringLiteral("texture_threshold_label"),
		QStringLiteral("texture_threshold_spin_box"),
		QStringLiteral("texture_threshold_slider"),
		0,256
		);

	construct_integer_control_set(
		uniqueness_ratio_horizontal_layout,
		uniqueness_ratio_label,
		uniqueness_ratio_spin_box,
		uniqueness_ratio_slider,
		tr("uniqueness ratio:"),
		QStringLiteral("uniqueness_ratio_horizontal_layout"),
		QStringLiteral("uniqueness_ratio_label"),
		QStringLiteral("uniqueness_ratio_spin_box"),
		QStringLiteral("uniqueness_ratio_slider"),
		0,256
		);

}

tuning_panel_bm::~tuning_panel_bm(){
}

} /* namespace stereo_workbench */
} /* namespace reco */
