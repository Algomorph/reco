/*
 * macher_qt_wrapper.hpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
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
#include <QObject>
#include <opencv2/calib3d.hpp>
#include <reco/stereo_workbench/tuning_panel.hpp>

namespace reco {
namespace stereo_workbench {

class matcher_qt_wrapper_base: public QObject{
	Q_OBJECT
public:
	virtual void compute(const cv::Mat& left,const cv::Mat& right, cv::Mat& disparity) = 0;
	virtual int get_bock_size() const = 0;
	virtual int get_disparity_max_diff() const = 0;
	virtual int get_minimum_disparity() const = 0;
	virtual int get_num_disparities() const = 0;
	virtual int get_speckle_range() const = 0;
	virtual int get_speckle_window_size() const = 0;

	tuning_panel* panel;

public slots:
	//tuning slots
	virtual void set_block_size(int value) = 0;
	virtual void set_disparity_max_diff(int value) = 0;
	virtual void set_minimum_disparity(int value) = 0;
	virtual void set_num_disparities(int value) = 0;
	virtual void set_speckle_range(int value) = 0;
	virtual void set_speckle_window_size(int value) = 0;

signals:
	void parameters_changed();

};

template<class MATCHER>
class matcher_qt_wrapper : public matcher_qt_wrapper_base {
	static_assert(
        std::is_base_of<cv::StereoMatcher, MATCHER>::value,
        "MATCHER must be a descendant of cv::StereoMatcher"
    );
public:
	matcher_qt_wrapper(cv::Ptr<MATCHER> matcher);
	virtual ~matcher_qt_wrapper();

	virtual void compute(const cv::Mat& left,const cv::Mat& right, cv::Mat& disparity);

	virtual int get_bock_size() const;
	virtual int get_disparity_max_diff() const;
	virtual int get_minimum_disparity() const;
	virtual int get_num_disparities() const;
	virtual int get_speckle_range() const;
	virtual int get_speckle_window_size() const;


	//tuning slots
	virtual void set_block_size(int value);
	virtual void set_disparity_max_diff(int value);
	virtual void set_minimum_disparity(int value);
	virtual void set_num_disparities(int value);
	virtual void set_speckle_range(int value);
	virtual void set_speckle_window_size(int value);

protected:
	cv::Ptr<MATCHER> stereo_matcher;
};

} /* namespace stereo_workbench */
} /* namespace reco */

#include <reco/stereo_workbench/matcher_qt_wrapper.tpp>
