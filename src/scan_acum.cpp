#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>


namespace terrain_modelling
{

class Scan_Acum_Node : public rclcpp::Node
{
public:
  Scan_Acum_Node()
  : Node("scan_acum_node")
  {
    point_cloud_topic_ = declare_parameter("point_cloud_topic", "/livox/lidar");
    output_topic_ = declare_parameter("output_topic", "/terrain/scan_acum");
    map_frame_ = declare_parameter("map_frame", "odom");
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.1);
    num_frames_ = declare_parameter("num_frames", 5);
    // false: publica no map_frame. true: publica no frame do sensor do quadro mais novo
    // (necessario para o ground_segmentation, que calcula a altura do chao a partir do
    // frame da nuvem, e para o Patchwork++, que nao usa TF).
    publish_in_sensor_frame_ = declare_parameter("publish_in_sensor_frame", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Scan_Acum_Node::cloudCallback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "in=%s out=%s map_frame=%s num_frames=%d",
      point_cloud_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str(), num_frames_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    //Faz a transformada da nuvem de pontos para o frame do mapa, aloca em vetor Eigen
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

    // Acumula o quadro; so publica quando juntar num_frames_ quadros
    points_acum_.insert(points_acum_.end(), points.begin(), points.end());
    acum_counter_++;
    if (acum_counter_ < num_frames_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg->header.stamp;   // carimbo do quadro mais novo
    out.header.frame_id = publish_in_sensor_frame_ ? msg->header.frame_id : map_frame_;
    // map -> sensor no instante do quadro mais novo (identidade se publicar no map_frame)
    const Eigen::Isometry3d map_to_out =
      publish_in_sensor_frame_ ? cloud_to_map.inverse() : Eigen::Isometry3d::Identity();

    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");    // campos x, y, z em float32
    mod.resize(points_acum_.size());

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (const auto & p : points_acum_) {
      const Eigen::Vector3d q = map_to_out * p;
      *ox = static_cast<float>(q.x());
      *oy = static_cast<float>(q.y());
      *oz = static_cast<float>(q.z());
      ++ox; ++oy; ++oz;
    }
    out.is_dense = true;   // já filtramos NaN/inf
    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "publicados %zu pontos de %d quadros em %s",
      points_acum_.size(), acum_counter_, out.header.frame_id.c_str());

    // Limpa só depois de publicar
    points_acum_.clear();
    acum_counter_ = 0;
  }

  std::string point_cloud_topic_, output_topic_, map_frame_;
  double transform_tolerance_{0.1};
  int num_frames_{5};
  bool publish_in_sensor_frame_{true};

  int acum_counter_{0};
  std::vector<Eigen::Vector3d> points_acum_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace terrain_modelling

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<terrain_modelling::Scan_Acum_Node>());
  rclcpp::shutdown();
  return 0;
}
