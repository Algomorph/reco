//  ================================================================
//  Created by Gregory Kramida on 7/3/18.
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

#include <reco/math/math_typedefs.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>



namespace reco{
namespace voxels{

using namespace Eigen;

//TODO: when static inheritance makes it into C++, enforce a particular interface for these
struct voxel {
    static float initial_value() { return 1.0f;}
    static float value_to_float(float value) { return value;}
    static float float_to_value(float float_value) { return float_value;}

    union{
        float tsdf;
        float truncated_signed_distance_function;
    };
    union{
        float weight;
        float confidence_weight;
    };

    Vector3u color;
    Vector3f position;
    Vector3f rotation;

    voxel() :
            tsdf(voxel::value_to_float(voxel::initial_value())),
            weight(0),
            color(0,0,0),
            position(0.0f,0.0f,0.0f),
            rotation(0.0f,0.0f,0.0f)
    {}
};

}
}// namespace reco
