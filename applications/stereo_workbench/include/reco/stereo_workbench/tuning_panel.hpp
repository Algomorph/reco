/*
 * tuning_panel.hpp
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

#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>

namespace reco{

namespace stereo_workbench{

class tuning_panel : public QWidget{
public:
	tuning_panel(QWidget* parent = NULL);
	virtual ~tuning_panel();
protected:
	void construct_integer_control_set(
					QHBoxLayout*& layout,
					QLabel*& label,
					QSpinBox*& spin_box,
					QSlider*& slider,
					QString label_text,
					QString layout_name,
					QString label_name,
					QString spin_box_name,
					QString slider_name,
					int min_val, int max_val, int step = 1, int val = 0, int page_step = 1,
					QString slider_tooltip = "");

	void construct_double_control_set(
						QHBoxLayout*& layout,
						QLabel*& label,
						QDoubleSpinBox*& spin_box,
						QString label_text,
						QString layout_name,
						QString label_name,
						QString spin_box_name,
						double min_val, double max_val, double step = 1.0, double val = 0.0, double page_step = 1.0,
						QString tooltip = "");

	QVBoxLayout* tuning_controls_vlayout;


};


}//stereo_workbench

}//reco


