#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "terrain_modelling/terrain_model.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_core/iterators/SpiralIterator.hpp>

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
    sensor_frame_ = declare_parameter("sensor_frame", std::string("base_link"));
    transform_tolerance_ = declare_parameter("transform_tolerance", 0.1);

    // SELECAO DE METODO
    seg_ = declare_parameter("seg", std::string("gseg"));
    if (seg_ != "gseg" && seg_ != "patchwork" && seg_ != "csf") {
      RCLCPP_FATAL(get_logger(), "seg invalido: %s (use gseg, patchwork ou csf)", seg_.c_str());
      throw std::invalid_argument("seg invalido: " + seg_);
    }
    //false = metodos antigos
    use_csf_ = (seg_ == "csf");
    RCLCPP_INFO(get_logger(), "seg=%s use_csf=%s", seg_.c_str(), use_csf_ ? "true" : "false");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Sensor data QoS: best-effort, keep-last. A cloud publisher will not
    // match a default (reliable) subscription.
    grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TerrainModellingNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "in=%s out=%s map_frame=%s sensor_frame=%s",
      ground_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str(), sensor_frame_.c_str());
    //Arquivo da análise
    analysis_path_ = use_csf_ ? "analiseCSF.txt" : "analisegseg.txt";
    analysis_file_.open(analysis_path_, std::ios::app);
    if (!analysis_file_.is_open()) {
      RCLCPP_WARN(get_logger(), "nao foi possivel abrir %s", analysis_path_.c_str());
    }

    map_ = grid_map::GridMap({"elevation", "variance", "last_update"});
    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(32.0, 32.0), 0.3);
    // map_.setBasicLayers({"elevation"});
    //map_.add("elevation", 0.0);      // ou a altura do chão sob o robô na TF inicial
    map_.add("elevation", std::numeric_limits<float>::quiet_NaN()); // Inicializa com NaN para indicar que não há dados
    map_.add("variance", 1.0e4);     // σ ≈ 100 m  ->  K ≈ 1 na primeira medição
    map_.add("last_update", 0.0);

    t0_ = now();

    // Publica o mapa a cada 500 ms, independente da chegada de nuvens.
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
    // Leva a nuvem para o map_frame. Se ela ja vier no map_frame (ex.: scan_acum
    // publicando no odom), o TF e a identidade.
    Eigen::Isometry3d cloud_to_map;
    Eigen::Vector3d sensor_position;
    try {
      auto tf = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(transform_tolerance_));
      cloud_to_map = tf2::transformToEigen(tf.transform);

      auto tf_sensor = tf_buffer_->lookupTransform(
        map_frame_, sensor_frame_, msg->header.stamp,
        tf2::durationFromSec(transform_tolerance_));
      sensor_position = tf2::transformToEigen(tf_sensor.transform).translation();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s / %s -> %s failed: %s", msg->header.frame_id.c_str(), sensor_frame_.c_str(),
        map_frame_.c_str(), ex.what());
      return;
    }

    
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    std::vector<Eigen::Vector3d> points_map;

    points_map.reserve(msg->width * msg->height);
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      points_map.emplace_back(cloud_to_map * Eigen::Vector3d(*it_x, *it_y, *it_z));
    }

    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);
    std::vector<std::vector<std::vector<float>>> cell_z(
      rows, std::vector<std::vector<float>>(cols));

    // SEPARAR A POINT CLOUD NO GRID: guarda o Z de cada ponto na celula (x, y) correspondente.
    for (const auto & point : points_map) {

      grid_map::Index index;
      grid_map::Position position(point.x(), point.y());

      if (!map_.getIndex(position, index)) {
        continue;
      }

      cell_z[index(0)][index(1)].push_back(point.z());
    }

    //Pegar a MEDIANA por célula e atualizar kalman filter
    const double time_now = (rclcpp::Time(msg->header.stamp) - t0_).seconds();

    //Referencias diretas as camadas (evita busca por string a cada acesso) e constantes do filtro.
    auto & elevation = map_["elevation"];
    auto & variance = map_["variance"];
    auto & last_update = map_["last_update"];

    // [CSF vs gseg] Minimo de pontos por celula: CSF aceita 1 ponto; gseg/patchwork exigem 2.
    const size_t min_points = use_csf_ ? 1 : 2;

    // Altura representativa da celula: mediana (gseg/patchwork) ou minimo (CSF).
    double median = 0.0;
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {

        auto & z_values = cell_z[i][j];
        if (z_values.empty()) {
          continue;
        }

        const size_t n = z_values.size();
        if (n < min_points) {
          continue;
        }

        // [CSF] A malha ja esta rente ao chao, entao a altura da celula e o menor Z.
        if (use_csf_) {
          median = *std::min_element(z_values.begin(), z_values.end());
        } else {
          // [GSEG/PATCHWORK] Altura da celula = mediana dos Z (robusta a outliers).
          if (n % 2 == 1) {//ÍMPAR
            std::nth_element(z_values.begin(), z_values.begin() + n / 2, z_values.end());
            median = z_values[n / 2];
          } else {//PAR
            std::nth_element(z_values.begin(), z_values.begin() + n / 2, z_values.end());
            double median2 = z_values[n / 2];
            double median1 = *std::max_element(z_values.begin(), z_values.begin() + n / 2);
            median = (median1 + median2) / 2.0;
          }
        }

        //KALMAN UPDATE
        // Posicao da celula e distancia ate o sensor (descarta celula colada no sensor).
        grid_map::Index index = grid_map::Index(i, j);
        grid_map::Position cell_position;
        map_.getPosition(index, cell_position);

        const double distance =
          (Eigen::Vector3d(cell_position.x(), cell_position.y(), median) - sensor_position).norm();
        if (distance < 1.0e-3) {
          continue;
        }

        //r_k: incerteza da medicao por propagacao de erro (depende do angulo e da distancia).
        // Para entender melhor o r_k,  ler propagação de erro em https://infoscience.epfl.ch/server/api/core/bitstreams/20ca2fc1-b9b7-4316-a30a-938cef8b00a8/content
        const double sigma_p = 0.03;
        const double sigma_B = 0.0;
        const float q = 0.00001f;
        const double l = std::hypot(
          cell_position.x() - sensor_position.x(), cell_position.y() - sensor_position.y());
        const double B = std::acos(std::clamp(l / distance, -1.0, 1.0));
        const float r_k = std::pow(std::sin(B), 2) * std::pow(sigma_p, 2) +
          std::pow(distance * std::cos(B), 2) * std::pow(sigma_B, 2);

        //Estatisticas dos vizinhos em espiral (raio 1.5 m): media, soma dos quadrados e variancia.
        double elevation_mean = 0;
        double variance_mean = 0.0;
        double elevation_sq_sum = 0.0;
        int counter = 0;
        for (grid_map::SpiralIterator it(map_, cell_position, 1.5); !it.isPastEnd(); ++it) {
          const grid_map::Index neighbor_index = *it;
          if ((neighbor_index == index).all()) {
            continue; // Pula a célula central
          }
          const double neighbor_elevation = elevation(neighbor_index(0), neighbor_index(1));
          if (!std::isnan(neighbor_elevation)) {
            elevation_mean += neighbor_elevation;
            elevation_sq_sum += neighbor_elevation * neighbor_elevation;
            variance_mean += variance(neighbor_index(0), neighbor_index(1));
            counter++;
          }
        }
        if (distance > 5.0 && counter < 5) {
          continue; //Descarta a medição se estiver muito longe e não houver vizinhos suficientes
        }

       
        if (counter > 3) { //FILTRO DE PICOS
          elevation_mean = elevation_mean / counter;
          double variancia_terreno =
            std::max(0.0, elevation_sq_sum / counter - std::pow(elevation_mean, 2)); // Variância do terreno = E[X^2] - (E[X])^2
          variance_mean = variance_mean / counter;
          double S = r_k + variance_mean + variancia_terreno + std::pow(0.05, 2);
          double nu = (median - elevation_mean); //Quanto a medição distoa dos vizinhos

          if (nu > 0 && nu / (std::sqrt(S)) > 2.0) { //Descarta se a medição estiver mais de 2 desvios acima da média dos vizinhos
            continue;
          }
        } else {
          // elevation_mean = median; // Se não houver vizinhos válidos, mantém a mediana calculada
          // variance_mean = r_k; // Variância da medição
          elevation_mean = 0.0; // Se não houver vizinhos válidos, descarta a medição
        }

        // Primeira medicao da celula: inicializa elevacao, variancia e instante da ultima atualizacao.
        if (std::isnan(elevation(i, j))) {
          elevation(i, j) = elevation_mean;
          if (counter > 3) {
            variance(i, j) = variance_mean + 0.03;
          }
          last_update(i, j) = static_cast<float>(time_now);
        }

        // Kalman propaga a estima a posteriori (variancia cresce com o tempo, q).
        float x_k_1 = elevation(i, j);
        float p_k_1 = variance(i, j);

        float x_k_priori = x_k_1;

        const double time_since_last_update =
          std::max(0.0, time_now - static_cast<double>(last_update(i, j)));

        float p_k_priori = p_k_1 + q * time_since_last_update;

        // atualizacao com a medicao (mediana ou minimo da celula).
        float K = p_k_priori / (r_k + p_k_priori);
        float x_k_posteriori = x_k_priori + K * (median - x_k_priori);
        float p_k_posteriori = (1 - K) * p_k_priori * (1 - K) + K * r_k * K;

        elevation(i, j) = x_k_posteriori;
        variance(i, j) = p_k_posteriori;
        last_update(i, j) = static_cast<float>(time_now);
      }
    }

    // Diagnostico: quantas celulas tem elevacao valida e quantas tem variancia NaN.
    const int valid = static_cast<int>(elevation.size() - elevation.array().isNaN().count());
    const int nan_var = static_cast<int>(variance.array().isNaN().count());
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "pontos=%zu validas=%d var_nan=%d seg=%s", points_map.size(), valid, nan_var, seg_.c_str());

    // Registro em arquivo: acumula 5 frames do scan_acum e grava uma linha por lote.
    if (batch_frames_ == 0) {
      batch_t_start_ = time_now;
    }
    batch_points_ += points_map.size();
    batch_frames_++;

    if (batch_frames_ >= kAnalysisFrames) {
      if (analysis_file_.is_open()) {
        analysis_file_ << std::fixed << std::setprecision(3)
          << "t_ini=" << batch_t_start_ << " t_fim=" << time_now
          << " frames=" << batch_frames_
          << " pontos=" << batch_points_
          << " validas=" << valid
          << " var_nan=" << nan_var
          << " seg=" << seg_ << std::endl;
      }
      batch_frames_ = 0;
      batch_points_ = 0;
    }
  }

  std::string ground_topic_, output_topic_, map_frame_, sensor_frame_;
  double transform_tolerance_{0.1};

  // [SELECAO DE METODO] Nome do metodo e flag do CSF.
  std::string seg_;
  bool use_csf_{false};

  static constexpr int kAnalysisFrames{5};
  std::string analysis_path_;
  std::ofstream analysis_file_;
  int batch_frames_{0};
  size_t batch_points_{0};
  double batch_t_start_{0.0};


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