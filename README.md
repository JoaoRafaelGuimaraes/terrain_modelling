# terrain_modelling

Modelo de elevação do terreno (DEM) em ROS 2 Jazzy. Recebe os pontos de chão do [`ground_segmentation_ros2`](https://github.com/dfki-ric/ground_segmentation_ros2) e mantém um `grid_map` com a altura de cada célula, atualizada por um filtro de Kalman.

O cenário de teste é o LIMO Pro com Livox MID-360 no mundo Cerrado do Gazebo Harmonic: [Limo_Jazzy_Cerrado](https://github.com/JoaoRafaelGuimaraes/Limo_Jazzy_Cerrado).

```
Gazebo (limo_ws) ──/livox/lidar, /livox/imu──▶ ground_segmentation_ros2 ──/ground_segmentation/ground_points──▶ terrain_modelling ──GridMap──▶ RViz
```

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy ([instalação](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
- Pacotes do sistema:

```bash
sudo apt update
sudo apt install git git-lfs curl tmux tmuxinator \
  python3-colcon-common-extensions \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 ros-jazzy-depth-image-proc ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-grid-map-core ros-jazzy-grid-map-ros ros-jazzy-grid-map-msgs \
  ros-jazzy-grid-map-rviz-plugin \
  ros-jazzy-pcl-conversions ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-message-filters \
  libpcl-dev libeigen3-dev libopencv-dev libnanoflann-dev libgtest-dev
```

`ros-jazzy-ros-gz` já instala o Gazebo Harmonic.

## Instalação

O `session.yml` usa os caminhos `/root/limo_ws` e `/root/mrs_ws`. Para usar outros, edite as linhas `source` do `session.yml`.

### 1. Simulação (limo_ws)

```bash
git clone --recursive https://github.com/JoaoRafaelGuimaraes/Limo_Jazzy_Cerrado.git /root/limo_ws
cd /root/limo_ws
git lfs install && git lfs pull      # meshes do terreno e do robô
./build.sh                           # também baixa o padrão de varredura do MID-360
```

### 2. Segmentação de chão + este pacote

```bash
mkdir -p /root/mrs_ws/src && cd /root/mrs_ws/src
git clone https://github.com/dfki-ric/ground_segmentation.git
git clone https://github.com/dfki-ric/ground_segmentation_ros2.git
git clone git@github.com:JoaoRafaelGuimaraes/terrain_modelling.git

cd /root/mrs_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to terrain_modelling ground_segmentation_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Configuração do tmux

O `session.yml` carrega `/etc/ctu-mrs/tmux.conf`, que vem com o [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system). Se ele não estiver instalado, crie um arquivo vazio:

```bash
sudo mkdir -p /etc/ctu-mrs && sudo touch /etc/ctu-mrs/tmux.conf
```

## Execução

```bash
cd /root/mrs_ws/src/terrain_modelling
./start.sh
```

Abre a sessão tmux `simulation` (socket `DEM_Simulation`) com uma janela por processo:

| Janela | O que roda |
|---|---|
| `simulation_stack` | Gazebo + LIMO Pro no Cerrado (`limo_cerrado.launch.py`) |
| `teleop` | `teleop_twist_keyboard`, janela inicial. Dirija o robô por aqui. |
| `ground_segmentation` | `ground_segmentation_ros2` sobre `/livox/lidar` + `/livox/imu` |
| `terrain_modelling` | este nó |

Para encerrar tudo:

```bash
tmux -L DEM_Simulation kill-server
```

Para rodar só o nó, com a segmentação já no ar:

```bash
ros2 launch terrain_modelling terrain_modelling.launch.py use_sim_time:=true
```

## Nó `terrain_modelling_node`

### Funcionamento

1. Transforma cada nuvem de chão para `map_frame` via TF, no carimbo da mensagem.
2. Distribui os pontos num grid de 32 × 32 m com células de 0,3 m, centrado na origem de `map_frame`.
3. Em cada célula que recebeu pontos, toma a mediana de z como medição.
4. Atualiza a célula com um filtro de Kalman escalar. O estado é a altura. A variância cresce com o tempo desde a última atualização (q = 1e-5 por segundo). O ruído de medição é fixo em r = 0,1.
5. Publica o mapa a 2 Hz.

### Tópicos

| Tópico | Tipo | Direção |
|---|---|---|
| `ground_topic` | `sensor_msgs/PointCloud2` | entrada (QoS sensor data) |
| `output_topic` | `grid_map_msgs/GridMap` | saída (QoS sensor data) |

Camadas do `GridMap`:

| Camada | Conteúdo |
|---|---|
| `elevation` | altura estimada do terreno [m] |
| `variance` | variância da estimativa [m²]. Começa em 1e4 nas células nunca vistas. |
| `last_update` | instante da última medição [s desde o início do nó] |

### Parâmetros

Arquivo: [`config/terrain_modelling.yaml`](config/terrain_modelling.yaml). Outro arquivo pode ser passado com `config_yaml:=`.

| Parâmetro | Padrão no código | Descrição |
|---|---|---|
| `ground_topic` | `/ground_segmentation/ground_points` | nuvem de chão |
| `output_topic` | `/terrain/grid_map` | saída do `GridMap` |
| `map_frame` | `odom` | referencial do grid. Ver "Referencial" abaixo. |
| `transform_tolerance` | `0.1` | espera máxima pelo TF [s] |

Os outros campos do YAML (grid, preenchimento, suavização) ainda não são lidos pelo nó. Por enquanto, a geometria do grid está fixa no código.

### Referencial (`map_frame`)

O nó usa o TF para saber onde o robô está e quanto ele está inclinado. Se o TF não tiver z, roll e pitch, a inclinação do robô aparece como relevo falso no mapa. Por isso o `map_frame` precisa ser 3D e alinhado à gravidade:

- **Simulação:** use `odom`. No `limo_ws`, o TF `odom -> base_footprint` já vem com a pose 3D exata do Gazebo.
- **Robô real:** use o frame de odometria de um SLAM 3D com IMU, como um LIO (no FAST-LIO2, `camera_init`). Odometria de rodas e SLAM 2D não servem, porque não medem a inclinação.

### Visualização

Rode a simulação com `rviz:=true`. A configuração do RViz do `limo_ws` já mostra o DEM e os pontos de chão.

Em outro RViz, adicione o display **GridMap** (do `grid_map_rviz_plugin`) no tópico `output_topic`, com QoS **Best Effort** e a camada `elevation`.

## Licença

BSD-3-Clause
