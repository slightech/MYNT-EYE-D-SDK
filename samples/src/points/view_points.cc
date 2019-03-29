#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "util/optparse.h"

using point_t = pcl::PointXYZRGBA;
using pointcloud_t = pcl::PointCloud<point_t>;

std::shared_ptr<pcl::visualization::PCLVisualizer> CustomColorVis(
    pointcloud_t::ConstPtr cloud, int xw, int yw) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud<point_t>(cloud, "points");
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer->setSize(xw, yw);
  return (viewer);
}

int main(int argc, char const *argv[]) {
  optparse::OptionParser parser = optparse::OptionParser()
      .usage("usage: %prog [options]"
      "\n  help: %prog -h"
      "\n  view: %prog -f *.ply"
      )
      .description("View point cloud data file.");

  parser.add_option("-f", "--file").dest("file")
      .type("string").metavar("FILE")
      .help("The point cloud data file");
  parser.add_option("-w", "--vw").dest("view_width")
      .type("int").set_default(1280)
      .metavar("WIDTH").help("The window width in pixels, default: %default");
  parser.add_option("-h", "--vh").dest("view_height")
      .type("int").set_default(720)
      .metavar("HEIGHT").help("The window height in pixels, default: %default");

  auto&& options = parser.parse_args(argc, argv);
  // auto&& args = parser.args();

  std::string filename = options["file"];
  if (filename.empty()) {
    std::cerr << "Please specify the file with -f option!" << std::endl;
    return 2;
  }

  std::cout << "Open file " << filename << std::endl;

  pointcloud_t::Ptr cloud(new pointcloud_t);
  int ret = pcl::io::loadPLYFile(filename, *cloud);
  if (ret != 0) {
    std::cerr << "Load the file (" << filename << ") failed!" << std::endl;
    return 1;
  }

  int w = options.get("view_width");
  int h = options.get("view_height");
  auto viewer = CustomColorVis(cloud, w, h);
  pcl::visualization::PointCloudColorHandlerRGBField<point_t>color(cloud);
  viewer->updatePointCloud<point_t>(cloud, color, "points");
  viewer->spin();

  return 0;
}
