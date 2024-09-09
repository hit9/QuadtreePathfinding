#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>

#include "visualizer.hpp"

CommandlineOptions options;

// Parse options from command line.
// Returns 0 on success.
int ParseCommandlineOptions(int argc, char* argv[]);

int main(int argc, char* argv[]) {
  if (0 != ParseCommandlineOptions(argc, argv)) return -1;
  Visualizer visualizer(options.w, options.h, options.gridSize, options.step);
  if (0 != visualizer.Init()) return -1;
  visualizer.Start();
  visualizer.Destroy();
  return 0;
}

int ParseCommandlineOptions(int argc, char* argv[]) {
  argparse::ArgumentParser program("quadtree-pathfinding-visualizer");
  program.add_argument("-w", "--width")
      .help("width of grid map")
      .default_value(70)
      .store_into(options.w);
  program.add_argument("-h", "--height")
      .help("height of grid map")
      .default_value(50)
      .store_into(options.h);
  program.add_argument("-gs", "--grid-cell-size-in-pixels")
      .help("grid cell size in pixels")
      .default_value(18)
      .store_into(options.gridSize);
  program.add_argument("-step")
      .help("the step to pick gates")
      .default_value(-1)
      .store_into(options.step);
  program.add_argument("-fonts", "--fonts-dir-path")
      .help("the relative fonts dir path")
      .default_value(std::string("./visualizer/fonts"))
      .store_into(options.fontsPath);
  program.add_argument("-cf", "--clearance-field")
      .help(
          "the clearance field implementer to use: 0 for TrueClearanceField, 1 for "
          "BrushfireClearanceField")
      .default_value(0)
      .store_into(options.clearanceFieldFlag);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& e) {
    spdlog::error(e.what());
    return 1;
  }
  if (options.w > N || options.h > N) {
    spdlog::error("w or h is too large");
    return 2;
  }
  return 0;
}
