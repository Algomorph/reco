//  ================================================================
//  Created by Gregory Kramida on 7/4/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once

//stdlib
#include <vector>

//Eigen
#include <Eigen/Dense>

//OpenCV
#include <opencv2/videoio.hpp>

//VTK
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkRenderer.h>

using namespace Eigen;

class scene_update_command;

class volume_deform_application {
public:
    friend class scene_update_command;
    volume_deform_application();

private:
    void set_up_main_window();
    void set_up_window_interactor();

    void add_layer(const Vector4d &backgroundColor = Vector4d(1.0, 1.0, 1.0, 1.0));
    void add_actor_to_layer(vtkSmartPointer<vtkActor> actor, int layer);
    void add_actor2D_to_layer(vtkSmartPointer<vtkActor2D> actor, int layer);
    void add_actor_to_first_layer(vtkSmartPointer<vtkActor> actor);

    void process_frame();

    vtkSmartPointer<vtkRenderWindow> main_window;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    std::vector<vtkSmartPointer<vtkRenderer>> layer_renderers;

    cv::VideoCapture depth_capture;
    cv::VideoCapture color_capture;
    cv::Mat depth_frame;
    cv::Mat color_frame;
};


