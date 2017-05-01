#include <fstream>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

DEFINE_string(in_cloud_xyz, "", "");
DEFINE_string(out_cloud_pcd, "", "");

bool loadCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::ifstream fs;
  fs.open(filename.c_str(), std::ios::binary);
  if(!fs.is_open() || fs.fail()) {
    PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
    fs.close();
    return false;
  }
  std::string line;
  std::vector<std::string> st;

  while(!fs.eof()) {
    std::getline(fs, line);
    if(line == "") {
      continue;
    }
    boost::trim(line);
    boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
    if(st.size () != 3) {
      continue;
    }
    cloud.push_back(pcl::PointXYZ(static_cast<float>(atof(st[0].c_str())), static_cast<float>(atof(st[1].c_str())), static_cast<float>(atof(st[2].c_str()))));
  }
  fs.close();

  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return true;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Load the .xyz-file.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if(!loadCloud(FLAGS_in_cloud_xyz, cloud)) {
    return (-1);
  }

  // Convert to .pcd and save.
  pcl::PCDWriter writer;
  writer.writeBinaryCompressed(FLAGS_out_cloud_pcd, cloud);

  return 0;
}
