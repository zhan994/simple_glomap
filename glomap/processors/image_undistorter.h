#ifndef PROCESSORS_IMAGE_UNDISTORTER_H
#define PROCESSORS_IMAGE_UNDISTORTER_H

#include "glomap/scene/types_sfm.h"

namespace glomap {

// api: 图像去畸变
void UndistortImages(std::unordered_map<camera_t, Camera> &cameras,
                     std::unordered_map<image_t, Image> &images,
                     bool clean_points = true);

} // namespace glomap

#endif // PROCESSORS_IMAGE_UNDISTORTER_H