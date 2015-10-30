/*
 * stereo_matcher_tuning_panel.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 */

#include <reco/stereo_workbench/stereo_matcher_tuning_panel.hpp>
#include <QVBoxLayout>
#include <reco/utils/debug_util.h>
#include <reco/utils/cpp_exception_util.h>
#include <reco/stereo_workbench/matcher_qt_wrapper_bm.hpp>
#include <reco/stereo_workbench/matcher_qt_wrapper_sgbm.hpp>

namespace reco {
namespace stereo_workbench {


stereo_matcher_tuning_panel::stereo_matcher_tuning_panel(QWidget* parent):
		tuning_panel(parent),
		matchers({std::shared_ptr<matcher_qt_wrapper_base>(new matcher_qt_wrapper_sgbm()),
				          std::shared_ptr<matcher_qt_wrapper_base>(new matcher_qt_wrapper_bm()),
						  std::shared_ptr<matcher_qt_wrapper_base>()}),
		specialized_parameter_panel(NULL)
		{

	other_controls_vlayout = new QVBoxLayout();
	this->layout()->addItem(other_controls_vlayout);
	set_up_tuning_controls();

}

void stereo_matcher_tuning_panel
	::connect_stereo_processor(stereo_processor& processor){
	this->processor = &processor;
	std::shared_ptr<matcher_qt_wrapper_base> matcher = processor.get_matcher();
	rectify_checkbox->setChecked(processor.is_rectification_enabled());

	//connect processor itself
	v_offset_slider->               setValue(processor.get_v_offset());
	v_offset_spin_box->             setValue(processor.get_v_offset());
	connect(v_offset_slider, SIGNAL(valueChanged(int)), &processor, SLOT(set_v_offset(int)));


	connect(&processor, SIGNAL(matcher_updated(matcher_qt_wrapper_base*)),
			this, SLOT(connect_matcher(matcher_qt_wrapper_base*)));
	connect_matcher(matcher.get());


}

void stereo_matcher_tuning_panel::swap_specialized_panel(tuning_panel* new_panel){
	if(specialized_parameter_panel){
		tuning_controls_vlayout->removeWidget(specialized_parameter_panel);
	}
	specialized_parameter_panel = new_panel;
	new_panel->setParent(this);
	new_panel->setVisible(true);


	this->layout()->addWidget(specialized_parameter_panel);
}
void stereo_matcher_tuning_panel::connect_matcher(matcher_qt_wrapper_base* matcher){
	//connect matcher
	//=== set values
	//sliders
	minimum_disparity_slider->      setValue(matcher->get_minimum_disparity());
	number_of_disparities_slider->  setValue(matcher->get_num_disparities());
	block_size_slider->             setValue(matcher->get_bock_size());
	speckle_window_size_slider->    setValue(matcher->get_speckle_window_size());
	speckle_range_slider->          setValue(matcher->get_speckle_range());

	//spin-boxes
	minimum_disparity_spin_box->    setValue(matcher->get_minimum_disparity());
	number_of_disparities_spin_box->setValue(matcher->get_num_disparities());
	block_size_spin_box->           setValue(matcher->get_bock_size());
	speckle_window_size_spin_box->  setValue(matcher->get_speckle_window_size());
	speckle_range_spin_box->        setValue(matcher->get_speckle_range());

	//disconnect from previous (if any)
	disconnect(minimum_disparity_slider, 0,0,0);
	disconnect(number_of_disparities_slider, 0,0,0);
	disconnect(block_size_slider, 0,0,0);
	disconnect(speckle_window_size_slider, 0,0,0);
	disconnect(speckle_range_slider, 0,0,0);

	//connect controls
	connect(minimum_disparity_slider, SIGNAL(valueChanged(int)), matcher, SLOT(set_minimum_disparity(int)));
	connect(number_of_disparities_slider, SIGNAL(valueChanged(int)), matcher, SLOT(set_num_disparities(int)));
	connect(block_size_slider, SIGNAL(valueChanged(int)), matcher, SLOT(set_block_size(int)));
	connect(speckle_window_size_slider, SIGNAL(valueChanged(int)), matcher, SLOT(set_speckle_window_size(int)));
	connect(speckle_range_slider, SIGNAL(valueChanged(int)), matcher, SLOT(set_speckle_range(int)));
	swap_specialized_panel(matcher->panel);
}



void stereo_matcher_tuning_panel::set_up_tuning_controls(){

	construct_integer_control_set(
			block_size_horizontal_layout,
			block_size_label,
			block_size_spin_box,
			block_size_slider,
			tr("block size:"),
			QStringLiteral("block_size_horizontal_layout"),
			QStringLiteral("block_size_label"),
			QStringLiteral("block_size_spin_box"),
			QStringLiteral("block_size_slider"),
			1,65,2,1
			);

	construct_integer_control_set(
			disparity_max_diff_horizontal_layout,
			disparity_max_diff_label,
			disparity_max_diff_spin_box,
			disparity_max_diff_slider,
			tr("max. pixel diff.:"),
			QStringLiteral("disparity_max_diff_horizontal_layout"),
			QStringLiteral("disparity_max_diff_label"),
			QStringLiteral("disparity_max_diff_spin_box"),
			QStringLiteral("disparity_max_diff_slider"),
			0,256
			);

	construct_integer_control_set(
			minimum_disparity_horizontal_layout,
			minimum_disparity_label,
			minimum_disparity_spin_box,
			minimum_disparity_slider,
			tr("minimum disparity:"),
			QStringLiteral("minimum_disparity_horizontal_layout"),
			QStringLiteral("minimum_disparity_label"),
			QStringLiteral("minimum_disparity_spin_box"),
			QStringLiteral("minimum_disparity_slider"),
			0,256
			);

	construct_integer_control_set(
			number_of_disparities_horizontal_layout,
			number_of_disparities_label,
			number_of_disparities_spin_box,
			number_of_disparities_slider,
			tr("number of disparities:"),
			QStringLiteral("number_of_disparities_horizontal_layout"),
			QStringLiteral("number_of_disparities_label"),
			QStringLiteral("number_of_disparities_spin_box"),
			QStringLiteral("number_of_disparities_slider"),
			16,256,16,16
			);

	construct_integer_control_set(
			speckle_range_horizontal_layout,
			speckle_range_label,
			speckle_range_spin_box,
			speckle_range_slider,
			tr("speckle range:"),
			QStringLiteral("speckle_range_horizontal_layout"),
			QStringLiteral("speckle_range_label"),
			QStringLiteral("speckle_range_spin_box"),
			QStringLiteral("speckle_range_slider"),
			0,5
			);

	construct_integer_control_set(
			speckle_window_size_horizontal_layout,
			speckle_window_size_label,
			speckle_window_size_spin_box,
			speckle_window_size_slider,
			tr("speckle window size:"),
			QStringLiteral("speckle_window_size_horizontal_layout"),
			QStringLiteral("speckle_window_size_label"),
			QStringLiteral("speckle_window_size_spin_box"),
			QStringLiteral("speckle_window_size_slider"),
			0,300,5,0,20
			);

	construct_integer_control_set(
			v_offset_horizontal_layout,
			v_offset_label,
			v_offset_spin_box,
			v_offset_slider,
			tr("vertical offset (px):"),
			QStringLiteral("v_offset_horizontal_layout"),
			QStringLiteral("v_offset_label"),
			QStringLiteral("v_offset_spin_box"),
			QStringLiteral("v_offset_slider"),
			-128,128
			);

	rectification_horizontal_layout = new QHBoxLayout();
	rectification_horizontal_layout->setObjectName(QStringLiteral("rectification_horizontal_layout"));
	rectify_checkbox = new QCheckBox(this);
	rectify_checkbox->setObjectName(QStringLiteral("rectify_checkbox"));
	rectify_checkbox->setChecked(true);
	rectify_checkbox->setText("enable rectification");

	rectification_horizontal_layout->addWidget(rectify_checkbox);


	other_controls_vlayout->addLayout(rectification_horizontal_layout);

	save_current_button = new QPushButton(this);
	save_current_button->setObjectName(QStringLiteral("save_current_button"));
	save_current_button->setText("Save current");
	save_current_button->setToolTip("save current stereo matcher input");

	other_controls_vlayout->addWidget(save_current_button);

	matcher_type_combo_box = new QComboBox(this);
	matcher_type_combo_box->setObjectName(QStringLiteral("matcher_type_combo_box"));
	other_controls_vlayout->addWidget(matcher_type_combo_box);
	matcher_type_combo_box->clear();
	matcher_type_combo_box->insertItems(0, QStringList()
			<< QString("SGBM") << QString("BM") << QString("BP")
	);

	connect(matcher_type_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(on_matcher_type_combo_box_currentIndexChanged(int)));


}



void stereo_matcher_tuning_panel::on_matcher_type_combo_box_currentIndexChanged(int index){
	processor->set_matcher(matchers[index]);
}

stereo_matcher_tuning_panel::~stereo_matcher_tuning_panel(){

}



} /* namespace stereo_workbench */
} /* namespace reco */
