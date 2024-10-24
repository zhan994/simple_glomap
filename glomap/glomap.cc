#include "glomap/controllers/global_mapper.h"
#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_converter.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <colmap/util/logging.h>
#include <colmap/util/misc.h>
#include <colmap/util/timer.h>

#include <iostream>

namespace glomap {
// -------------------------------------
// api: Mappers starting from COLMAP database
// -------------------------------------
int RunMapper(int argc, char **argv) {
  std::string database_path;
  std::string output_path;

  std::string image_path = "";
  std::string constraint_type = "ONLY_POINTS";
  std::string output_format = "bin";

  // step: 1 参数初始化并解析
  OptionManager options;
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_path", &image_path);
  options.AddDefaultOption("constraint_type", &constraint_type,
                           "{ONLY_POINTS, ONLY_CAMERAS, "
                           "POINTS_AND_CAMERAS_BALANCED, POINTS_AND_CAMERAS}");
  options.AddDefaultOption("output_format", &output_format, "{bin, txt}");
  options.AddGlobalMapperFullOptions();
  options.Parse(argc, argv);

  // step: 2 检查database
  if (!colmap::ExistsFile(database_path)) {
    LOG(ERROR) << "`database_path` is not a file";
    return EXIT_FAILURE;
  }

  // step: 3 约束类型
  if (constraint_type == "ONLY_POINTS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_POINTS;
  } else if (constraint_type == "ONLY_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_CAMERAS;
  } else if (constraint_type == "POINTS_AND_CAMERAS_BALANCED") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED;
  } else if (constraint_type == "POINTS_AND_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS;
  } else {
    LOG(ERROR) << "Invalid constriant type";
    return EXIT_FAILURE;
  }

  // step: 4 检查输出格式
  if (output_format != "bin" && output_format != "txt") {
    LOG(ERROR) << "Invalid output format";
    return EXIT_FAILURE;
  }

  // step: 5 Load the database
  LOG(INFO) << "Loading Feature Database,,,";
  ViewGraph view_graph;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  const colmap::Database database(database_path);
  ConvertDatabaseToGlomap(database, view_graph, cameras, images);

  if (view_graph.image_pairs.empty()) {
    LOG(ERROR) << "Can't continue without image pairs";
    return EXIT_FAILURE;
  }

  // step: 6 solver的主入口
  GlobalMapper global_mapper(*options.mapper);
  LOG(INFO) << "Loaded database";
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(database, view_graph, cameras, images, tracks);
  run_timer.Pause();

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  return EXIT_SUCCESS;
}

} // namespace glomap

namespace {

typedef std::function<int(int, char **)> command_func_t;
int ShowHelp(
    const std::vector<std::pair<std::string, command_func_t>> &commands) {
  std::cout << "GLOMAP -- Global Structure-from-Motion" << std::endl
            << std::endl;

  std::cout << "Usage:" << std::endl;
  std::cout << "  glomap mapper --database_path DATABASE --output_path MODEL"
            << std::endl;
  std::cout << "  glomap mapper_resume --input_path MODEL_INPUT --output_path "
               "MODEL_OUTPUT"
            << std::endl;

  std::cout << "Available commands:" << std::endl;
  std::cout << "  help" << std::endl;
  for (const auto &command : commands) {
    std::cout << "  " << command.first << std::endl;
  }
  std::cout << std::endl;

  return EXIT_SUCCESS;
}

} // namespace

// api: 整个工程的主函数
int main(int argc, char **argv) {
  // step: 1 初始化日志
  colmap::InitializeGlog(argv);
  FLAGS_alsologtostderr = true;

  // step: 2 指令
  std::vector<std::pair<std::string, command_func_t>> commands;
  commands.emplace_back("mapper", &glomap::RunMapper);

  if (argc == 1) {
    return ShowHelp(commands);
  }

  // step: 3 指令解析并运行
  const std::string command = argv[1];
  if (command == "help" || command == "-h" || command == "--help") {
    return ShowHelp(commands);
  } else {
    command_func_t matched_command_func = nullptr;
    for (const auto &command_func : commands) {
      if (command == command_func.first) {
        matched_command_func = command_func.second;
        break;
      }
    }
    if (matched_command_func == nullptr) {
      std::cout << "Command " << command << " not recognized. "
                << "To list the available commands, run `colmap help`."
                << std::endl;
      return EXIT_FAILURE;
    } else {
      int command_argc = argc - 1;
      char **command_argv = &argv[1];
      command_argv[0] = argv[0];
      return matched_command_func(command_argc, command_argv);
    }
  }

  return ShowHelp(commands);
}
