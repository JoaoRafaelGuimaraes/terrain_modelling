#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "terrain_modelling/terrain_model.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>


using namespace grid_map;


namespace terrain_modelling
{

class TerrainModellingNode : public rclcpp::Node
{
public:
  TerrainModellingNode()
  : Node("terrain_modelling_node")
  {
    ground_topic_ = declare_parameter("ground_topic", "/ground_segmentation/ground_points");
    output_topic_ = declare_parameter("output_topic", "/terrain/grid_map");
    map_frame_ = declare_parameter("map_frame", "odom");
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Sensor data QoS: best-effort, keep-last. A cloud publisher will not
    // match a default (reliable) subscription.
    grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainModellingNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "in=%s out=%s map_frame=%s",
      ground_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str());
    
    map_ = grid_map::GridMap({"elevation", "variance", "last_update"});
    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(32.0, 32.0), 0.3);
    // map_.setBasicLayers({"elevation"});

    map_.add("elevation", 0.0);      // ou a altura do chão sob o robô na TF inicial
    map_.add("variance", 1.0e4);     // σ ≈ 100 m  ->  K ≈ 1 na primeira medição
    map_.add("last_update", 0.0);

    t0_ = now();

    dem_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TerrainModellingNode::publishMap, this));
  }

  

private:

  void publishMap() //Transforma o mapa em PointCLoud2 para publicar
  { 
    map_.setTimestamp(now().nanoseconds());

    std::unique_ptr<grid_map_msgs::msg::GridMap> msg =
        grid_map::GridMapRosConverter::toMessage(map_);

    grid_map_pub_->publish(std::move(msg));
  }
  
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    
    Eigen::Isometry3d cloud_to_map;
    try {
      auto tf = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(transform_tolerance_));
      cloud_to_map = tf2::transformToEigen(tf.transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s -> %s failed: %s", msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    // PointCloud2Iterator walks the packed buffer by field name, so the node
    // does not care how the fields are laid out or whether PCL is around.
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->width * msg->height);
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      points.emplace_back(cloud_to_map * Eigen::Vector3d(*it_x, *it_y, *it_z));
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "got %zu points in %s", points.size(), map_frame_.c_str());

    
    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    std::vector<std::vector<std::vector<float>>> cell_z(
      rows, std::vector<std::vector<float>>(cols));

     // SEPARAR A POINT CLOUD NO GRID
    
    for (const auto & point : points) {

       grid_map::Index index;
       grid_map::Position position(point.x(), point.y());
       
       if (!map_.getIndex(position, index)) {
         continue;
       }

       cell_z[index(0)][index(1)].push_back(point.z()); //Adiciona o Z do ponto 
      
    }

    //Pegar a MEDIANA por célula e atualizar kalman filter
    const double time_now = (rclcpp::Time(msg->header.stamp) - t0_).seconds();

    double median = 0.0;
    for (int i = 0; i < rows; i++){
      for (int j = 0; j < cols; j++){
        
        auto & z_values = cell_z[i][j];
        if (z_values.empty()) {
          continue;
        }

        const size_t n = z_values.size();

        if (n % 2 == 1){ //ÍMPAR
          std::nth_element(z_values.begin(), z_values.begin() + n / 2, z_values.end());
          median = z_values[n / 2];
        } else { //PAR
          std::nth_element(z_values.begin(), z_values.begin() + n / 2 - 1, z_values.end());
          double median1 = z_values[n / 2 - 1];
          std::nth_element(z_values.begin(), z_values.begin() + n / 2, z_values.end());
          double median2 = z_values[n / 2];
          median = (median1 + median2) / 2.0;

        }

        //KALMAN UPDATE

        //step 1: PROPAGATE A POSTERIORI ESTIMATE
        grid_map::Index index = grid_map::Index(i,j);
        float x_k_1 = map_.at("elevation", index);
        float p_k_1 = map_.at("variance", index);

        float x_k_priori = x_k_1;
        
        const double time_since_last_update = std::max(0.0, time_now - static_cast<double>(map_.at("last_update", index)));

        float q = 0.00001; // Variância do processo, pode ser ajustada conforme necessário
        float p_k_priori = p_k_1 + q * time_since_last_update; //q é a variancia do processo. 
        // step 2: OBSERVATION UPDATE
      
        //R_k é a variancia da medição de Z. Pode ser calculada em função da distância do drone
        float r_k = 0.1; //Melhorar posteriormente
        float K = p_k_priori / (r_k + p_k_priori);
        float x_k_posteriori = x_k_priori + K * (median - x_k_priori);
        float p_k_posteriori = (1 - K) * p_k_priori * (1-K) + K*r_k*K; 

        map_.at("elevation", index) = x_k_posteriori;
        map_.at("variance", index) = p_k_posteriori;
        map_.at("last_update", index) = static_cast<float>(time_now);;

      }
    }

    
   


  }

  std::string ground_topic_, output_topic_, map_frame_;
  double transform_tolerance_{0.1};

  TerrainModel model_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  grid_map::GridMap map_;
  rclcpp::Time t0_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::TimerBase::SharedPtr dem_timer_;

};

}  // namespace terrain_modelling

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<terrain_modelling::TerrainModellingNode>());
  rclcpp::shutdown();
  return 0;
}
