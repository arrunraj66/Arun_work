#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>
#include <QWheelEvent>
#include <QWidget>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lidar/cloud_subscriber.hpp"
#include "lidar/point_cloud.hpp"

namespace {

struct RenderPoint {
  QVector3D position;
  QColor color;
};

class CloudView final : public QOpenGLWidget {
 public:
  explicit CloudView(QWidget* parent = nullptr) : QOpenGLWidget(parent) {
    setMinimumSize(760, 560);
    setMouseTracking(true);
  }

  void setPoints(std::vector<RenderPoint> points) {
    points_ = std::move(points);
    update();
  }

  void clearPoints() {
    points_.clear();
    update();
  }

  void setPointSize(int pixels) {
    point_size_ = pixels;
    update();
  }

  void frontView() {
    yaw_deg_ = 0.0F;
    pitch_deg_ = 0.0F;
    pan_ = QPointF{};
    update();
  }

  void sideView() {
    yaw_deg_ = 90.0F;
    pitch_deg_ = 0.0F;
    pan_ = QPointF{};
    update();
  }

  void topView() {
    yaw_deg_ = 0.0F;
    pitch_deg_ = 89.0F;
    pan_ = QPointF{};
    update();
  }

 protected:
  void paintGL() override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), QColor(4, 15, 27));

    drawGrid(painter);
    drawAxes(painter);

    painter.setPen(Qt::NoPen);
    for (const RenderPoint& point : points_) {
      QPointF screen;
      float depth = 0.0F;
      if (!project(point.position, screen, depth)) {
        continue;
      }
      painter.setBrush(point.color);
      painter.drawEllipse(screen, point_size_, point_size_);
    }

    painter.setPen(QColor(150, 190, 215));
    painter.drawText(12, 22, QStringLiteral("x: forward   y: left   z: up"));
    painter.drawText(12, 42,
                     QStringLiteral("Left mouse: orbit   Right mouse: pan   Wheel: zoom"));
  }

  void mousePressEvent(QMouseEvent* event) override {
    last_mouse_ = event->pos();
    event->accept();
  }

  void mouseMoveEvent(QMouseEvent* event) override {
    const QPoint delta = event->pos() - last_mouse_;
    last_mouse_ = event->pos();

    if ((event->buttons() & Qt::LeftButton) != 0) {
      yaw_deg_ += static_cast<float>(delta.x()) * 0.45F;
      pitch_deg_ = std::clamp(pitch_deg_ - static_cast<float>(delta.y()) * 0.45F,
                              -89.0F, 89.0F);
      update();
    } else if ((event->buttons() & Qt::RightButton) != 0) {
      pan_ += QPointF(delta);
      update();
    }
    event->accept();
  }

  void wheelEvent(QWheelEvent* event) override {
    const float steps = static_cast<float>(event->angleDelta().y()) / 120.0F;
    distance_ *= std::pow(0.85F, steps);
    distance_ = std::clamp(distance_, 1.0F, 500.0F);
    update();
    event->accept();
  }

 private:
  bool project(const QVector3D& world, QPointF& screen, float& depth) const {
    constexpr float pi = 3.14159265358979323846F;
    const float yaw = yaw_deg_ * pi / 180.0F;
    const float pitch = pitch_deg_ * pi / 180.0F;

    const QVector3D camera_position(
        distance_ * std::cos(pitch) * std::cos(yaw),
        distance_ * std::cos(pitch) * std::sin(yaw),
        distance_ * std::sin(pitch));

    const QVector3D forward = (-camera_position).normalized();
    QVector3D right = QVector3D::crossProduct(forward, QVector3D(0.0F, 0.0F, 1.0F));
    if (right.lengthSquared() < 0.0001F) {
      right = QVector3D(0.0F, 1.0F, 0.0F);
    } else {
      right.normalize();
    }
    const QVector3D up = QVector3D::crossProduct(right, forward).normalized();

    const QVector3D relative = world - camera_position;
    const float camera_x = QVector3D::dotProduct(relative, right);
    const float camera_y = QVector3D::dotProduct(relative, up);
    depth = QVector3D::dotProduct(relative, forward);
    if (depth <= 0.05F) {
      return false;
    }

    const float focal = 0.85F * static_cast<float>(std::min(width(), height()));
    screen.setX(static_cast<double>(width()) * 0.5 + pan_.x() +
                static_cast<double>(focal * camera_x / depth));
    screen.setY(static_cast<double>(height()) * 0.5 + pan_.y() -
                static_cast<double>(focal * camera_y / depth));
    return true;
  }

  void drawLine3D(QPainter& painter, const QVector3D& a, const QVector3D& b,
                  const QColor& color, int width = 1) const {
    QPointF pa;
    QPointF pb;
    float da = 0.0F;
    float db = 0.0F;
    if (!project(a, pa, da) || !project(b, pb, db)) {
      return;
    }
    QPen pen(color);
    pen.setWidth(width);
    painter.setPen(pen);
    painter.drawLine(pa, pb);
  }

  void drawGrid(QPainter& painter) const {
    const int half = 20;
    for (int value = -half; value <= half; value += 2) {
      const QColor color = value == 0 ? QColor(54, 91, 115) : QColor(25, 52, 70);
      drawLine3D(painter, QVector3D(static_cast<float>(value), -half, 0.0F),
                 QVector3D(static_cast<float>(value), half, 0.0F), color);
      drawLine3D(painter, QVector3D(-half, static_cast<float>(value), 0.0F),
                 QVector3D(half, static_cast<float>(value), 0.0F), color);
    }
  }

  void drawAxes(QPainter& painter) const {
    drawLine3D(painter, QVector3D(), QVector3D(3.0F, 0.0F, 0.0F),
               QColor(255, 90, 90), 2);
    drawLine3D(painter, QVector3D(), QVector3D(0.0F, 3.0F, 0.0F),
               QColor(90, 255, 130), 2);
    drawLine3D(painter, QVector3D(), QVector3D(0.0F, 0.0F, 3.0F),
               QColor(90, 150, 255), 2);
  }

  std::vector<RenderPoint> points_;
  QPoint last_mouse_;
  QPointF pan_;
  float yaw_deg_ = 35.0F;
  float pitch_deg_ = 24.0F;
  float distance_ = 30.0F;
  int point_size_ = 2;
};

struct ReceiverState {
  std::mutex mutex;
  lidar::PointCloud latest;
  std::uint64_t sequence = 0;
  std::uint64_t received = 0;
  std::uint64_t malformed = 0;
  std::size_t estimated_bytes = 0;
  QString error;
};

std::size_t estimatedPayloadBytes(const lidar::PointCloud& cloud) {
  const std::size_t floats = cloud.ranges.size() + cloud.azimuths.size() +
                             cloud.elevations.size() + cloud.intensities.size();
  return cloud.frame_id.size() + floats * sizeof(float) + 40U;
}

QColor gradient(float normalized) {
  const float value = std::clamp(normalized, 0.0F, 1.0F);
  const int hue = static_cast<int>((1.0F - value) * 240.0F);
  return QColor::fromHsv(hue, 235, 255);
}

class MainWindow final : public QMainWindow {
 public:
  MainWindow() {
    setWindowTitle(QStringLiteral("AUV Middleware — 3D LiDAR Viewer"));
    resize(1280, 760);

    auto* central = new QWidget(this);
    auto* root = new QHBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);

    view_ = new CloudView(central);
    root->addWidget(view_, 1);

    auto* controls = new QWidget(central);
    controls->setFixedWidth(330);
    auto* controls_layout = new QVBoxLayout(controls);

    auto* connection_group = new QGroupBox(QStringLiteral("Subscriber"), controls);
    auto* connection_form = new QFormLayout(connection_group);
    endpoint_ = new QLineEdit(QStringLiteral("tcp://127.0.0.1:5580"), connection_group);
    topic_ = new QLineEdit(QStringLiteral("lidar.cloud"), connection_group);
    connect_button_ = new QPushButton(QStringLiteral("Connect"), connection_group);
    pause_button_ = new QPushButton(QStringLiteral("Pause display"), connection_group);
    pause_button_->setCheckable(true);
    connection_form->addRow(QStringLiteral("Endpoint"), endpoint_);
    connection_form->addRow(QStringLiteral("Topic"), topic_);
    connection_form->addRow(connect_button_);
    connection_form->addRow(pause_button_);
    controls_layout->addWidget(connection_group);

    auto* display_group = new QGroupBox(QStringLiteral("Display"), controls);
    auto* display_form = new QFormLayout(display_group);
    color_mode_ = new QComboBox(display_group);
    color_mode_->addItems({QStringLiteral("Height"), QStringLiteral("Intensity"),
                           QStringLiteral("Range")});
    min_range_ = new QDoubleSpinBox(display_group);
    min_range_->setRange(0.0, 500.0);
    min_range_->setDecimals(2);
    min_range_->setValue(0.05);
    min_range_->setSuffix(QStringLiteral(" m"));
    max_range_ = new QDoubleSpinBox(display_group);
    max_range_->setRange(0.1, 500.0);
    max_range_->setDecimals(1);
    max_range_->setValue(30.0);
    max_range_->setSuffix(QStringLiteral(" m"));
    point_size_ = new QSlider(Qt::Horizontal, display_group);
    point_size_->setRange(1, 6);
    point_size_->setValue(2);
    display_form->addRow(QStringLiteral("Colour"), color_mode_);
    display_form->addRow(QStringLiteral("Minimum range"), min_range_);
    display_form->addRow(QStringLiteral("Maximum range"), max_range_);
    display_form->addRow(QStringLiteral("Point size"), point_size_);
    controls_layout->addWidget(display_group);

    auto* views_group = new QGroupBox(QStringLiteral("Camera"), controls);
    auto* views_layout = new QHBoxLayout(views_group);
    auto* front = new QPushButton(QStringLiteral("Front"), views_group);
    auto* side = new QPushButton(QStringLiteral("Side"), views_group);
    auto* top = new QPushButton(QStringLiteral("Top"), views_group);
    views_layout->addWidget(front);
    views_layout->addWidget(side);
    views_layout->addWidget(top);
    controls_layout->addWidget(views_group);

    auto* stats_group = new QGroupBox(QStringLiteral("Live statistics"), controls);
    auto* stats_form = new QFormLayout(stats_group);
    status_ = new QLabel(QStringLiteral("Disconnected"), stats_group);
    frame_id_ = new QLabel(QStringLiteral("—"), stats_group);
    points_ = new QLabel(QStringLiteral("0"), stats_group);
    rate_ = new QLabel(QStringLiteral("0.0 clouds/s"), stats_group);
    bytes_ = new QLabel(QStringLiteral("0 B"), stats_group);
    malformed_ = new QLabel(QStringLiteral("0"), stats_group);
    status_->setWordWrap(true);
    stats_form->addRow(QStringLiteral("Status"), status_);
    stats_form->addRow(QStringLiteral("Frame"), frame_id_);
    stats_form->addRow(QStringLiteral("Visible points"), points_);
    stats_form->addRow(QStringLiteral("Receive rate"), rate_);
    stats_form->addRow(QStringLiteral("Est. payload"), bytes_);
    stats_form->addRow(QStringLiteral("Malformed"), malformed_);
    controls_layout->addWidget(stats_group);

    auto* clear = new QPushButton(QStringLiteral("Clear view"), controls);
    controls_layout->addWidget(clear);
    controls_layout->addStretch(1);
    root->addWidget(controls);
    setCentralWidget(central);

    connect(connect_button_, &QPushButton::clicked, this, [this] { toggleConnection(); });
    connect(pause_button_, &QPushButton::toggled, this, [this](bool paused) {
      pause_button_->setText(paused ? QStringLiteral("Resume display")
                                   : QStringLiteral("Pause display"));
    });
    connect(point_size_, &QSlider::valueChanged, view_, &CloudView::setPointSize);
    connect(front, &QPushButton::clicked, view_, &CloudView::frontView);
    connect(side, &QPushButton::clicked, view_, &CloudView::sideView);
    connect(top, &QPushButton::clicked, view_, &CloudView::topView);
    connect(clear, &QPushButton::clicked, view_, &CloudView::clearPoints);
    connect(color_mode_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this] { rebuildLastCloud(); });
    connect(min_range_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this] { rebuildLastCloud(); });
    connect(max_range_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this] { rebuildLastCloud(); });

    refresh_timer_.setInterval(33);
    connect(&refresh_timer_, &QTimer::timeout, this, [this] { refresh(); });
    refresh_timer_.start();
    rate_clock_.start();
  }

  ~MainWindow() override { stopReceiver(); }

 private:
  void toggleConnection() {
    if (receiver_thread_.joinable()) {
      stopReceiver();
      status_->setText(QStringLiteral("Disconnected"));
      connect_button_->setText(QStringLiteral("Connect"));
      endpoint_->setEnabled(true);
      topic_->setEnabled(true);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.error.clear();
      state_.sequence = 0;
      state_.received = 0;
      state_.malformed = 0;
    }
    displayed_sequence_ = 0;
    previous_received_ = 0;
    rate_clock_.restart();
    stop_requested_.store(false);

    const std::string endpoint = endpoint_->text().trimmed().toStdString();
    const std::string topic = topic_->text().trimmed().toStdString();
    status_->setText(QStringLiteral("Connecting; waiting for point clouds…"));
    connect_button_->setText(QStringLiteral("Disconnect"));
    endpoint_->setEnabled(false);
    topic_->setEnabled(false);

    receiver_thread_ = std::thread([this, endpoint, topic] {
      try {
        lidar::CloudSubscriber subscriber(endpoint, topic);
        while (!stop_requested_.load()) {
          lidar::PointCloud cloud;
          if (!subscriber.poll_cloud(cloud, 100)) {
            continue;
          }

          const std::size_t bytes = estimatedPayloadBytes(cloud);
          std::lock_guard<std::mutex> lock(state_.mutex);
          state_.latest = std::move(cloud);
          ++state_.sequence;
          ++state_.received;
          state_.malformed = subscriber.malformed();
          state_.estimated_bytes = bytes;
        }
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(state_.mutex);
        state_.error = QString::fromUtf8(error.what());
      }
    });
  }

  void stopReceiver() {
    stop_requested_.store(true);
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

  void refresh() {
    lidar::PointCloud cloud;
    std::uint64_t sequence = 0;
    std::uint64_t received = 0;
    std::uint64_t malformed = 0;
    std::size_t bytes = 0;
    QString error;

    {
      std::lock_guard<std::mutex> lock(state_.mutex);
      sequence = state_.sequence;
      received = state_.received;
      malformed = state_.malformed;
      bytes = state_.estimated_bytes;
      error = state_.error;
      if (sequence != displayed_sequence_ && !pause_button_->isChecked()) {
        cloud = state_.latest;
      }
    }

    if (!error.isEmpty()) {
      status_->setText(QStringLiteral("Receiver error: ") + error);
    } else if (receiver_thread_.joinable() && received == 0) {
      status_->setText(QStringLiteral("Connected; waiting for point clouds…"));
    } else if (receiver_thread_.joinable()) {
      status_->setText(pause_button_->isChecked() ? QStringLiteral("Receiving; display paused")
                                                   : QStringLiteral("Receiving live data"));
    }

    if (!cloud.ranges.empty()) {
      last_cloud_ = std::move(cloud);
      displayed_sequence_ = sequence;
      rebuildLastCloud();
    }

    malformed_->setText(QString::number(static_cast<qulonglong>(malformed)));
    if (bytes >= 1024U * 1024U) {
      bytes_->setText(QString::number(static_cast<double>(bytes) / (1024.0 * 1024.0), 'f', 2) +
                      QStringLiteral(" MiB"));
    } else {
      bytes_->setText(QString::number(static_cast<double>(bytes) / 1024.0, 'f', 1) +
                      QStringLiteral(" KiB"));
    }

    if (rate_clock_.elapsed() >= 1000) {
      const double seconds = static_cast<double>(rate_clock_.elapsed()) / 1000.0;
      const double hz = static_cast<double>(received - previous_received_) / seconds;
      rate_->setText(QString::number(hz, 'f', 1) + QStringLiteral(" clouds/s"));
      previous_received_ = received;
      rate_clock_.restart();
    }
  }

  void rebuildLastCloud() {
    if (last_cloud_.ranges.empty()) {
      return;
    }

    const std::size_t count =
        std::min({last_cloud_.ranges.size(), last_cloud_.azimuths.size(),
                  last_cloud_.elevations.size()});
    const float minimum_range = static_cast<float>(min_range_->value());
    const float maximum_range = static_cast<float>(max_range_->value());

    std::vector<std::size_t> accepted;
    accepted.reserve(count);
    float minimum_height = std::numeric_limits<float>::max();
    float maximum_height = std::numeric_limits<float>::lowest();
    float minimum_intensity = std::numeric_limits<float>::max();
    float maximum_intensity = std::numeric_limits<float>::lowest();

    for (std::size_t i = 0; i < count; ++i) {
      const float range = last_cloud_.ranges[i];
      if (!std::isfinite(range) || range < minimum_range || range > maximum_range) {
        continue;
      }
      const lidar::Point3 xyz = lidar::to_xyz(last_cloud_, i);
      if (!std::isfinite(xyz.x) || !std::isfinite(xyz.y) || !std::isfinite(xyz.z)) {
        continue;
      }
      accepted.push_back(i);
      minimum_height = std::min(minimum_height, xyz.z);
      maximum_height = std::max(maximum_height, xyz.z);
      if (i < last_cloud_.intensities.size()) {
        minimum_intensity = std::min(minimum_intensity, last_cloud_.intensities[i]);
        maximum_intensity = std::max(maximum_intensity, last_cloud_.intensities[i]);
      }
    }

    std::vector<RenderPoint> render_points;
    render_points.reserve(accepted.size());
    for (const std::size_t i : accepted) {
      const lidar::Point3 xyz = lidar::to_xyz(last_cloud_, i);
      float value = 0.0F;
      if (color_mode_->currentIndex() == 0) {
        const float span = std::max(maximum_height - minimum_height, 0.0001F);
        value = (xyz.z - minimum_height) / span;
      } else if (color_mode_->currentIndex() == 1 &&
                 i < last_cloud_.intensities.size()) {
        const float span = std::max(maximum_intensity - minimum_intensity, 0.0001F);
        value = (last_cloud_.intensities[i] - minimum_intensity) / span;
      } else {
        const float span = std::max(maximum_range - minimum_range, 0.0001F);
        value = (last_cloud_.ranges[i] - minimum_range) / span;
      }
      render_points.push_back(
          RenderPoint{QVector3D(xyz.x, xyz.y, xyz.z), gradient(value)});
    }

    view_->setPoints(std::move(render_points));
    frame_id_->setText(QString::fromStdString(last_cloud_.frame_id));
    points_->setText(QStringLiteral("%1 / %2")
                         .arg(static_cast<qulonglong>(accepted.size()))
                         .arg(static_cast<qulonglong>(count)));
  }

  CloudView* view_ = nullptr;
  QLineEdit* endpoint_ = nullptr;
  QLineEdit* topic_ = nullptr;
  QPushButton* connect_button_ = nullptr;
  QPushButton* pause_button_ = nullptr;
  QComboBox* color_mode_ = nullptr;
  QDoubleSpinBox* min_range_ = nullptr;
  QDoubleSpinBox* max_range_ = nullptr;
  QSlider* point_size_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* frame_id_ = nullptr;
  QLabel* points_ = nullptr;
  QLabel* rate_ = nullptr;
  QLabel* bytes_ = nullptr;
  QLabel* malformed_ = nullptr;
  QTimer refresh_timer_;
  QElapsedTimer rate_clock_;

  ReceiverState state_;
  std::atomic<bool> stop_requested_{false};
  std::thread receiver_thread_;
  std::uint64_t displayed_sequence_ = 0;
  std::uint64_t previous_received_ = 0;
  lidar::PointCloud last_cloud_;
};

}  // namespace

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  QApplication::setApplicationName(QStringLiteral("AUV 3D LiDAR Viewer"));

  MainWindow window;
  window.show();
  return application.exec();
}
