// Independent PCL Target export; no SDK source or map payload embedded.
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 3 || std::filesystem::exists(argv[2])) {
    std::cerr << "Usage: map_export INPUT_PCD NEW_TARGET_BIN (output must not exist)\n";
    return 2;
  }
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(argv[1], *input) < 0) return 1;
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(.8f, .8f, .8f);
  pcl::PointCloud<pcl::PointXYZ> output;
  voxel.filter(output);
  std::ofstream file(argv[2], std::ios::binary);
  uint64_t count = output.size();
  file.write(reinterpret_cast<char*>(&count), 8);
  for (auto& point : output) {
    float xyz[3] = {point.x, point.y, point.z};
    file.write(reinterpret_cast<char*>(xyz), 12);
  }
  return !file;
}
