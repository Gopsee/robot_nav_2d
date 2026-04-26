#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

// ── RS-16 constants ──────────────────────────────────────────────────────────
static constexpr int    NUM_CHANNELS   = 16;
static constexpr int    NUM_HORIZONTAL = 1800;
static constexpr double H_RES_RAD      = 0.2  * M_PI / 180.0;
static constexpr double MAX_RANGE      = 150.0;
static constexpr double MIN_RANGE      = 0.4;
static constexpr double RANGE_NOISE_STD = 0.02;   // 2 cm
static constexpr double DROPOUT_PROB    = 0.02;
static constexpr double SENSOR_Z        = 1.0;    // lidar height above ground (m)

static const double ELEVATIONS[NUM_CHANNELS] = {
  -15, -13, -11, -9, -7, -5, -3, -1,
    1,   3,   5,  7,  9, 11, 13, 15
};

// ── PointCloud2 layout: x y z intensity ring + 2-byte pad = 20 bytes ────────
static constexpr uint32_t POINT_STEP = 20;
static constexpr uint32_t OFF_X = 0, OFF_Y = 4, OFF_Z = 8, OFF_I = 12, OFF_R = 16;

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node"),
    noise_gen_(std::random_device{}()),
    noise_(0.0, RANGE_NOISE_STD),
    drop_dist_(0.0, 1.0)
  {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10,
      std::bind(&LidarNode::poseCallback, this, std::placeholders::_1));

    scan_pub_  = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);

    // Transient Local so RViz gets the markers even if it connects later
    auto latched = rclcpp::QoS(1).transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", latched);

    buildWorld();         // populates world_, top_faces_, obstacles_
    buildCloudTemplate();
    publishMarkers();     // one-shot latched publish

    RCLCPP_INFO(get_logger(),
      "RS-16 lidar started: %d channels × %d pts, range %.1f–%.1f m, "
      "%zu obstacles",
      NUM_CHANNELS, NUM_HORIZONTAL, MIN_RANGE, MAX_RANGE, obstacles_.size());
  }

private:
  // ── World geometry ────────────────────────────────────────────────────────

  struct Segment {
    double x1, y1, x2, y2;
    double max_z;   // wall extends from z=0 to max_z; -1 = infinite
  };

  struct TopFace {
    double cx, cy, hw, hd, z;   // axis-aligned rectangle at height z
  };

  struct Obstacle {
    double cx, cy, hw, hd, height;
    float r, g, b;              // display color
  };

  std::vector<Segment> world_;
  std::vector<TopFace> top_faces_;
  std::vector<Obstacle> obstacles_;
  sensor_msgs::msg::PointCloud2 cloud_tmpl_;

  // Pose history for rolling-shutter interpolation
  double prev_x_{0}, prev_y_{0}, prev_theta_{0};
  double curr_x_{0}, curr_y_{0}, curr_theta_{0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::mt19937 noise_gen_;
  std::normal_distribution<double> noise_;
  std::uniform_real_distribution<double> drop_dist_;

  // ── Build world ───────────────────────────────────────────────────────────

  void buildWorld()
  {
    // Boundary walls (infinite height)
    constexpr double H = 200.0;
    for (auto & seg : std::vector<Segment>{
        {-H, -H,  H, -H, -1},
        { H, -H,  H,  H, -1},
        { H,  H, -H,  H, -1},
        {-H,  H, -H, -H, -1},
    }) world_.push_back(seg);

    // Random box obstacles — fixed seed so layout is the same every launch
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> pos(-40.0, 40.0);
    std::uniform_real_distribution<double> half_sz(0.5, 2.5);
    std::uniform_real_distribution<double> ht_dist(1.0, 5.0);
    // Colour palette (HSV-inspired, deterministic)
    const float palette[][3] = {
      {0.85f,0.33f,0.25f}, {0.20f,0.60f,0.85f}, {0.30f,0.75f,0.40f},
      {0.90f,0.60f,0.15f}, {0.55f,0.25f,0.80f}, {0.10f,0.70f,0.70f},
      {0.95f,0.45f,0.55f}, {0.45f,0.80f,0.20f}, {0.70f,0.20f,0.60f},
      {0.20f,0.45f,0.90f},
    };

    for (int i = 0; i < 25; ) {
      double cx = pos(rng), cy = pos(rng);
      if (cx * cx + cy * cy < 16.0) continue;   // keep 4 m clear around origin

      double hw = half_sz(rng);
      double hd = half_sz(rng);
      double height = ht_dist(rng);
      const auto & col = palette[i % 10];

      obstacles_.push_back({cx, cy, hw, hd, height, col[0], col[1], col[2]});

      // 4 finite-height walls
      world_.push_back({cx - hw, cy - hd, cx + hw, cy - hd, height});
      world_.push_back({cx + hw, cy - hd, cx + hw, cy + hd, height});
      world_.push_back({cx + hw, cy + hd, cx - hw, cy + hd, height});
      world_.push_back({cx - hw, cy + hd, cx - hw, cy - hd, height});

      // Top face (hit by upward beams)
      top_faces_.push_back({cx, cy, hw, hd, height});

      ++i;
    }
  }

  // ── PointCloud2 field layout ──────────────────────────────────────────────

  void buildCloudTemplate()
  {
    auto add = [&](const std::string & n, uint32_t off, uint8_t type) {
      sensor_msgs::msg::PointField f;
      f.name = n; f.offset = off; f.datatype = type; f.count = 1;
      cloud_tmpl_.fields.push_back(f);
    };
    add("x",         OFF_X, sensor_msgs::msg::PointField::FLOAT32);
    add("y",         OFF_Y, sensor_msgs::msg::PointField::FLOAT32);
    add("z",         OFF_Z, sensor_msgs::msg::PointField::FLOAT32);
    add("intensity", OFF_I, sensor_msgs::msg::PointField::FLOAT32);
    add("ring",      OFF_R, sensor_msgs::msg::PointField::UINT16);

    cloud_tmpl_.header.frame_id = "map";
    cloud_tmpl_.height     = 1;
    cloud_tmpl_.is_dense   = true;
    cloud_tmpl_.point_step = POINT_STEP;
  }

  // ── Ray casting ───────────────────────────────────────────────────────────

  double intersect2D(double ox, double oy, double dx, double dy,
                     const Segment & s) const
  {
    double ex = s.x2 - s.x1, ey = s.y2 - s.y1;
    double denom = dx * ey - dy * ex;
    if (std::fabs(denom) < 1e-8) return INFINITY;
    double fx = s.x1 - ox, fy = s.y1 - oy;
    double t = (fx * ey - fy * ex) / denom;
    double u = (fx * dy - fy * dx) / denom;
    return (t >= MIN_RANGE && u >= 0.0 && u <= 1.0) ? t : INFINITY;
  }

  double castRay3D(double ox, double oy, double oz,
                   double dx, double dy, double dz) const
  {
    double min_t = MAX_RANGE;

    // Vertical walls (height-limited for finite obstacles)
    for (const auto & s : world_) {
      double t = intersect2D(ox, oy, dx, dy, s);
      if (t >= min_t) continue;
      double z_hit = oz + t * dz;
      if (z_hit < 0.0) continue;                      // underground
      if (s.max_z > 0.0 && z_hit > s.max_z) continue; // ray passes over top
      min_t = t;
    }

    // Top faces of obstacles (hit by upward-angled beams)
    if (dz > 1e-6) {
      for (const auto & f : top_faces_) {
        double t = (f.z - oz) / dz;
        if (t < MIN_RANGE || t >= min_t) continue;
        double hx = ox + t * dx, hy = oy + t * dy;
        if (std::fabs(hx - f.cx) <= f.hw && std::fabs(hy - f.cy) <= f.hd) {
          min_t = t;
        }
      }
    }

    // Ground plane z = 0
    if (dz < -1e-6) {
      double t = -oz / dz;
      if (t > MIN_RANGE && t < min_t) min_t = t;
    }

    return min_t;
  }

  // ── Pose callback ─────────────────────────────────────────────────────────

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    prev_x_ = curr_x_; prev_y_ = curr_y_; prev_theta_ = curr_theta_;
    curr_x_ = msg->pose.position.x;
    curr_y_ = msg->pose.position.y;
    curr_theta_ = 2.0 * std::atan2(msg->pose.orientation.z, msg->pose.orientation.w);
    publishCloud(msg->header.stamp);
  }

  void interpPose(double t, double & x, double & y, double & theta) const
  {
    x     = prev_x_     + t * (curr_x_     - prev_x_);
    y     = prev_y_     + t * (curr_y_     - prev_y_);
    theta = prev_theta_ + t * (curr_theta_ - prev_theta_);
  }

  // ── Publish point cloud ───────────────────────────────────────────────────

  void publishCloud(const rclcpp::Time & stamp)
  {
    std::vector<uint8_t> buf;
    buf.reserve(NUM_CHANNELS * NUM_HORIZONTAL * POINT_STEP / 4); // ~25% hit rate
    uint32_t count = 0;

    for (int h = 0; h < NUM_HORIZONTAL; ++h) {
      double t_frac = static_cast<double>(h) / NUM_HORIZONTAL;
      double rx, ry, rtheta;
      interpPose(t_frac, rx, ry, rtheta);
      double az = rtheta + h * H_RES_RAD;

      for (int c = 0; c < NUM_CHANNELS; ++c) {
        if (drop_dist_(noise_gen_) < DROPOUT_PROB) continue;

        double el_rad = ELEVATIONS[c] * M_PI / 180.0;
        double cos_el = std::cos(el_rad);
        double sin_el = std::sin(el_rad);
        double dx = cos_el * std::cos(az);
        double dy = cos_el * std::sin(az);
        double dz = sin_el;

        double r = castRay3D(rx, ry, SENSOR_Z, dx, dy, dz) + noise_(noise_gen_);
        if (r >= MAX_RANGE) continue;

        float fx = static_cast<float>(rx + r * dx);
        float fy = static_cast<float>(ry + r * dy);
        float fz = static_cast<float>(SENSOR_Z + r * dz);
        float fi = static_cast<float>(1.0 - r / MAX_RANGE);
        auto  fr = static_cast<uint16_t>(c);

        buf.resize(buf.size() + POINT_STEP);
        uint8_t * p = buf.data() + count * POINT_STEP;
        std::memcpy(p + OFF_X, &fx, 4);
        std::memcpy(p + OFF_Y, &fy, 4);
        std::memcpy(p + OFF_Z, &fz, 4);
        std::memcpy(p + OFF_I, &fi, 4);
        std::memcpy(p + OFF_R, &fr, 2);
        ++count;
      }
    }

    auto cloud        = cloud_tmpl_;
    cloud.header.stamp = stamp;
    cloud.width       = count;
    cloud.row_step    = count * POINT_STEP;
    cloud.data        = std::move(buf);
    cloud_pub_->publish(cloud);
  }

  // ── Publish obstacle markers (called once at startup) ─────────────────────

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;

    for (std::size_t i = 0; i < obstacles_.size(); ++i) {
      const auto & ob = obstacles_[i];

      visualization_msgs::msg::Marker m;
      m.header.frame_id    = "map";
      m.header.stamp       = rclcpp::Time(0);
      m.ns                 = "obstacles";
      m.id                 = static_cast<int>(i);
      m.type               = visualization_msgs::msg::Marker::CUBE;
      m.action             = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x    = ob.cx;
      m.pose.position.y    = ob.cy;
      m.pose.position.z    = ob.height * 0.5;   // centre vertically
      m.pose.orientation.w = 1.0;
      m.scale.x            = ob.hw * 2.0;
      m.scale.y            = ob.hd * 2.0;
      m.scale.z            = ob.height;
      m.color.r            = ob.r;
      m.color.g            = ob.g;
      m.color.b            = ob.b;
      m.color.a            = 0.80f;
      m.lifetime           = rclcpp::Duration(0, 0);   // never expire

      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
