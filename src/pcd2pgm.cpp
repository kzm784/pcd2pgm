#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Pcd2Pgm : public rclcpp::Node
{
public:
  Pcd2Pgm() : Node("pcd2pgm")
  {
    // Parameters
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("min_points_in_pix", 2);
    this->declare_parameter("max_points_in_pix", 5);
    this->declare_parameter("min_height", 0.5);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("input_pcd", "");
    this->declare_parameter("dest_directory", "");
    this->declare_parameter("output_pgm_name", "");

    this->get_parameter("resolution", resolution);
    this->get_parameter("min_points_in_pix", min_points_in_pix);
    this->get_parameter("max_points_in_pix", max_points_in_pix);
    this->get_parameter("min_height", min_height);
    this->get_parameter("max_height", max_height);
    this->get_parameter("input_pcd", input_pcd);
    this->get_parameter("dest_directory", dest_directory);
    this->get_parameter("output_pgm_name", output_pgm_name);

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(input_pcd, *cloud_xyz) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read the PCD file");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Initial point cloud size: %lu", cloud_xyz->points.size());

    // Determine map size and origin
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto& point : *cloud_xyz)
    {
      if (point.z >= min_height && point.z <= max_height) {
        x_min = std::min(x_min, (double)point.x);
        x_max = std::max(x_max, (double)point.x);
        y_min = std::min(y_min, (double)point.y);
        y_max = std::max(y_max, (double)point.y);
      }
    }

    map_width = std::ceil((x_max - x_min) / resolution);
    map_height = std::ceil((y_max - y_min) / resolution);

    RCLCPP_INFO(this->get_logger(), "Map size: width = %d, height = %d", map_width, map_height);

    // Create 2D map
    cv::Mat map = generate(*cloud_xyz, x_min, y_min);

    // Flip the map to correct the orientation
    cv::flip(map, map, 0);  // 0 means flipping around the x-axis (i.e., vertical flip)

    if (!boost::filesystem::exists(dest_directory))
    {
      boost::filesystem::create_directories(dest_directory);
    }

    // Save as PGM file
    std::string pgm_file_path = dest_directory + "/" + output_pgm_name + ".pgm";
    cv::imwrite(pgm_file_path, map);

    // Save the YAML file with corrected origin
    std::string yaml_file_path = dest_directory + "/" + output_pgm_name + ".yaml";
    std::ofstream ofs(yaml_file_path);
    ofs << "image: " << output_pgm_name << ".pgm" << std::endl;
    ofs << "resolution: " << resolution << std::endl;
    ofs << "origin: [" << x_min << ", " << (y_max - map_height * resolution) << ", 0.0]" << std::endl;
    ofs << "occupied_thresh: 0.5" << std::endl;
    ofs << "free_thresh: 0.2" << std::endl;
    ofs << "negate: 0" << std::endl;

    // Log the successful save
    RCLCPP_INFO(this->get_logger(), "Map saved as PGM file: %s", pgm_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "YAML file saved: %s", yaml_file_path.c_str());
  }

private:
  double resolution;
  int map_width;
  int map_height;
  int min_points_in_pix;
  int max_points_in_pix;
  double min_height;
  double max_height;
  std::string input_pcd;
  std::string dest_directory;
  std::string output_pgm_name;

  cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ>& cloud, double x_min, double y_min) const
  {
    cv::Mat map(map_height, map_width, CV_32SC1, cv::Scalar::all(0));
    double m2pix = 1.0 / resolution;

    for (const auto& point : cloud)
    {
      if (point.z < min_height || point.z > max_height)
      {
        continue;
      }

      int x = std::round((point.x - x_min) * m2pix);
      int y = std::round((point.y - y_min) * m2pix);

      if (x < 0 || x >= map_width || y < 0 || y >= map_height)
      {
        continue;
      }

      map.at<int>(y, x)++;
    }

    map -= min_points_in_pix;
    map = cv::max(map, 0);
    map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);

    return map;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pcd2Pgm>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}

