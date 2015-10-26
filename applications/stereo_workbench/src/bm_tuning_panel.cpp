/*
 * bm_tuning_panel.cpp
 *
 *  Created on: Oct 26, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/bm_tuning_panel.hpp>

namespace reco {
namespace stereo_workbench {

bm_tuning_panel::bm_tuning_panel(QWidget* parent):stereo_matcher_tuning_panel<stereo_processor_bm>(parent){
	construct_specialized_controls();
}

void bm_tuning_panel::construct_specialized_controls(){
	pre_filter_type_combo_box = new QComboBox(central_widget);
	pre_filter_type_combo_box->setObjectName(QStringLiteral("pre_filter_type_combo_box"));
	tuning_controls_vlayout->addWidget(pre_filter_type_combo_box);
	pre_filter_type_combo_box->clear();
	pre_filter_type_combo_box->insertItems(0, QStringList()
	 << QApplication::translate("main_window", "normalized response", 0)
	 << QApplication::translate("main_window", "x-sobel", 0)
	);
}

bm_tuning_panel::~bm_tuning_panel(){
	// TODO Auto-generated destructor stub
}

} /* namespace stereo_workbench */
} /* namespace reco */
