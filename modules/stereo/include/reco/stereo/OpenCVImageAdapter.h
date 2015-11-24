/*
 * OpenCV2ImageAdapter.h
 *
 *  Created on: Nov 20, 2015
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

#include <opencv2/core.hpp>
#include "OvImageAdapter.h"

///Provides an interface to images from the OpenCV library.
/**
* The OpenCVImageAdapter class is inherited from the OvImageAdapter and
* provides a wrapper around OpenCV's IplImage allowing the OpenVis3D library functions to access the
* dimensions and datatype of the image, and provides get and
* set methods to alter image pixels.
*
* @see OvImageAdapter
*
* @author Abhijit Ogale
*/
class OpenCVImageAdapter :	public OvImageAdapter
{
  OpenCVImageAdapter(){}; /**< to prevent the default constructor from being used */

public:
  OpenCVImageAdapter(cv::Mat im);
  virtual ~OpenCVImageAdapter();

  virtual double getPixel(int row, int column, int channel) const;
  virtual void   setPixel(double value, int row, int column, int channel);

protected:
  cv::Mat mat;	/**< saved OpenCV matrix object */

  double (OpenCVImageAdapter::*getPixelfptr) (int row, int column, int channel) const; /**< function pointer used to store getpixel function appropriate for image datatype */
  void   (OpenCVImageAdapter::*setPixelfptr) (double value, int row, int column, int channel); /**< function pointer used to store setpixel function appropriate for image datatype */

  template<typename T> double getPixelT(int row, int column, int channel) const;
  template<typename T> void   setPixelT(double value, int row, int column, int channel);

  double getPixeldoNothing(int row, int column, int channel) const;
  void   setPixeldoNothing(double value, int row, int column, int channel);
};


