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

//VTK
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>

//utils
#include <reco/utils/cpp_exception_util.h>

//local
#include "volume_deform_application.h"

class scene_update_command : public vtkCommand {

public:
    vtkTypeMacro(scene_update_command, vtkCommand);
    static scene_update_command* New(){
        return new scene_update_command;
    }
    void Execute(vtkObject*vtkNotUsed(caller), unsigned long vtkNotUsed(eventId),
                 void*vtkNotUsed(callData)) override {
        parent->process_frame();
    }
    volume_deform_application* parent = nullptr;
};

volume_deform_application::volume_deform_application() :
        main_window(vtkSmartPointer<vtkRenderWindow>::New()),
        interactor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
        depth_capture("/media/algomorph/Data/Reconstruction/real_data/KillingFusion Snoopy/frames/depth_%06d.png"),
        color_capture("/media/algomorph/Data/Reconstruction/real_data/KillingFusion Snoopy/frames/color_%06d.png"),
        depth_frame(),
        color_frame()

    {
    set_up_main_window();
    set_up_window_interactor();
    add_layer();

    interactor->Start();
}

void volume_deform_application::set_up_main_window() {
    main_window->SetWindowName("Volume Deform");
    main_window->SetSize(main_window->GetScreenSize());
}

void volume_deform_application::set_up_window_interactor() {
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor->SetInteractorStyle(interactorStyle);
    interactor->SetRenderWindow(main_window);
    interactor->Initialize();
    interactor->CreateRepeatingTimer(1);
    vtkSmartPointer<scene_update_command> update_callback = vtkSmartPointer<scene_update_command>::New();
    update_callback->parent = this;
    interactor->AddObserver(vtkCommand::TimerEvent, update_callback);
}


void volume_deform_application::add_layer(const Vector4d &backgroundColor) {
    vtkSmartPointer<vtkRenderer> new_renderer = vtkSmartPointer<vtkRenderer>::New();
    new_renderer->SetBackground(backgroundColor[0], backgroundColor[1], backgroundColor[2]);
    new_renderer->SetBackgroundAlpha(backgroundColor[3]);
    new_renderer->SetLayer(static_cast<int>(layer_renderers.size()));
    if (!layer_renderers.empty()) {
        new_renderer->SetActiveCamera(layer_renderers[0]->GetActiveCamera());
    }
    main_window->SetNumberOfLayers(static_cast<int>(layer_renderers.size() + 1));
    main_window->AddRenderer(new_renderer);
    layer_renderers.push_back(new_renderer);
}


void volume_deform_application::add_actor_to_layer(vtkSmartPointer<vtkActor> actor, int layer) {
    if (layer < 0 || layer >= main_window->GetNumberOfLayers()) {
        err3("Layer " + std::to_string(layer) + " out of bounds.");
    }
    layer_renderers[layer]->AddActor(actor);
}

void volume_deform_application::add_actor2D_to_layer(vtkSmartPointer<vtkActor2D> actor, int layer) {
    if (layer < 0 || layer >= main_window->GetNumberOfLayers()) {
        err3("Layer " + std::to_string(layer) + " out of bounds.");
    }
    layer_renderers[layer]->AddActor2D(actor);
}

void volume_deform_application::add_actor_to_first_layer(vtkSmartPointer<vtkActor> actor) {
    main_window->GetRenderers()->GetFirstRenderer()->AddActor(actor);
}

void volume_deform_application::process_frame() {
    if(depth_capture.read(depth_frame) && color_capture.read(color_frame));
}
