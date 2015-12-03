/*
 * tuning_panel.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

#include <reco/stereo_tuner/tuning_panel.hpp>


namespace reco{

namespace stereo_tuner{

tuning_panel ::
	tuning_panel(QWidget* parent): QWidget(parent){
	QVBoxLayout* layout = new QVBoxLayout();
	this->setLayout(layout);
	tuning_controls_vlayout = new QVBoxLayout();
	layout->addItem(tuning_controls_vlayout);

}
tuning_panel ::~tuning_panel(){}


void tuning_panel::construct_integer_control_set(
			QHBoxLayout*& horizontal_layout,
			QLabel*& label,
			QSpinBox*& spin_box,
			QSlider*& slider,
			QString label_text,
			QString layout_name,
			QString label_name,
			QString spin_box_name,
			QString slider_name,
			int min_val, int max_val, int step, int val, int page_step,
			QString slider_tooltip){
	horizontal_layout = new QHBoxLayout();
	horizontal_layout->setObjectName(layout_name);
	label = new QLabel(this);
	label->setObjectName(label_name);
	label->setText(label_text);

	horizontal_layout->addWidget(label);

	spin_box = new QSpinBox(this);
	spin_box->setObjectName(spin_box_name);
	spin_box->setMinimum(min_val);
	spin_box->setMaximum(max_val);
	spin_box->setSingleStep(step);


	horizontal_layout->addWidget(spin_box);

	tuning_controls_vlayout->addLayout(horizontal_layout);

	slider = new QSlider(this);
	slider->setObjectName(slider_name);
	QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
	sizePolicy.setHorizontalStretch(0);
	sizePolicy.setVerticalStretch(0);
	sizePolicy.setHeightForWidth(slider->sizePolicy().hasHeightForWidth());
	slider->setSizePolicy(sizePolicy);
	slider->setMinimumSize(QSize(400, 0));
	slider->setMinimum(min_val);
	slider->setMaximum(max_val);
	slider->setSingleStep(step);
	slider->setPageStep(page_step);
	slider->setValue(val);
	slider->setTracking(false);
	slider->setOrientation(Qt::Horizontal);
	if(!slider_tooltip.isEmpty()){
		slider->setToolTip(slider_tooltip);
	}
	tuning_controls_vlayout->addWidget(slider);

	//connect slider and spinbox
	//TODO: investigate - this causes double value upgrades during spinbox stepping
	connect(slider, SIGNAL(valueChanged(int)), spin_box, SLOT(setValue(int)));
	connect(spin_box, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
}

void tuning_panel::construct_double_control_set(
			QHBoxLayout*& horizontal_layout,
			QLabel*& label,
			QDoubleSpinBox*& spin_box,
			QString label_text,
			QString layout_name,
			QString label_name,
			QString spin_box_name,
			double min_val, double max_val, double step, double val, double page_step,
			QString tooltip){

	horizontal_layout = new QHBoxLayout();
	horizontal_layout->setObjectName(layout_name);
	label = new QLabel(this);
	label->setObjectName(label_name);
	label->setText(label_text);

	horizontal_layout->addWidget(label);

	spin_box = new QDoubleSpinBox(this);
	spin_box->setObjectName(spin_box_name);
	spin_box->setMinimum(min_val);
	spin_box->setMaximum(max_val);
	spin_box->setSingleStep(step);
	spin_box->setToolTip(tooltip);


	horizontal_layout->addWidget(spin_box);

	tuning_controls_vlayout->addLayout(horizontal_layout);

}


}//stereo_tuner

}//reco



