//Copyright 2018. Tingfung Lau. All Rights Reserverd

#pragma once

#include "common/proto/object_labeling_3d.pb.h"
#include "common/proto/perception.pb.h"
#include "homework4/camera_lidar_fusion_utils.h"

#include <vector>

namespace perception {

interface::perception::PerceptionObstacles build(const PointCloud &pointcloud, 
	const interface::object_labeling::ObjectLabels &label);

}