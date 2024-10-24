#ifndef IO_COLMAP_CONVERTER_H
#define IO_COLMAP_CONVERTER_H

#include "glomap/scene/types_sfm.h"

#include <colmap/scene/database.h>
#include <colmap/scene/image.h>

namespace glomap {

// api: 将colmap的数据库转为view graph
void ConvertDatabaseToGlomap(const colmap::Database &database,
                             ViewGraph &view_graph,
                             std::unordered_map<camera_t, Camera> &cameras,
                             std::unordered_map<image_t, Image> &images);

} // namespace glomap

#endif // IO_COLMAP_CONVERTER_H