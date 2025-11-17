#include "joint_jogger_gui/joint_jogger_panel.hpp"

#include <QHBoxLayout>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <algorithm>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

namespace joint_jogger_gui
{

JointJoggerPanel::JointJoggerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout(this);

  // Controls box
  auto * ctrl_box = new QGroupBox("Settings", this);
  auto * ctrl_layout = new QGridLayout(ctrl_box);
  int r = 0;

  // Topic
  ctrl_layout->addWidget(new QLabel("Topic:"), r, 0);
  topic_edit_ = new QLineEdit(QString::fromStdString(topic_), ctrl_box);
  ctrl_layout->addWidget(topic_edit_, r, 1, 1, 3);
  connect(topic_edit_, &QLineEdit::editingFinished, this, &JointJoggerPanel::onTopicEdited);
  r++;

  // Rate
  ctrl_layout->addWidget(new QLabel("Rate (Hz):"), r, 0);
  rate_spin_ = new QDoubleSpinBox(ctrl_box);
  rate_spin_->setRange(1.0, 200.0);
  rate_spin_->setDecimals(1);
  rate_spin_->setValue(rate_hz_);
  ctrl_layout->addWidget(rate_spin_, r, 1);
  connect(
    rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    &JointJoggerPanel::onRateChanged);

  // Joint count + +/- buttons
  count_spin_ = new QSpinBox(ctrl_box);
  count_spin_->setRange(1, 64);
  count_spin_->setValue(joint_count_);
  connect(
    count_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
    &JointJoggerPanel::onCountChanged);

  add_one_btn_ = new QPushButton("+1", ctrl_box);
  remove_one_btn_ = new QPushButton("-1", ctrl_box);
  connect(add_one_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onAddOne);
  connect(remove_one_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onRemoveOne);

  ctrl_layout->addWidget(new QLabel("Joints:"), r, 2);
  ctrl_layout->addWidget(count_spin_, r, 3);
  r++;

  ctrl_layout->addWidget(add_one_btn_, r, 2);
  ctrl_layout->addWidget(remove_one_btn_, r, 3);
  r++;

  // Stop all
  stop_all_btn_ = new QPushButton("Stop All", ctrl_box);
  connect(stop_all_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onStopAll);
  ctrl_layout->addWidget(stop_all_btn_, r, 0, 1, 4);
  r++;
  
  // Publish toggle button
  toggle_btn_ = new QPushButton("OFF", ctrl_box);
  toggle_btn_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
  ctrl_layout->addWidget(toggle_btn_, r, 0, 1, 4);
  connect(toggle_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onTogglePublishing);
  r++;

  ctrl_box->setLayout(ctrl_layout);
  root_layout->addWidget(ctrl_box);

  // Joints scroll area
  joints_container_ = new QWidget(this);
  joints_grid_ = new QGridLayout(joints_container_);
  joints_grid_->setColumnStretch(1, 1);  // – column
  joints_grid_->setColumnStretch(2, 1);  // + column

  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(joints_container_);
  root_layout->addWidget(scroll, 1);

  // Data init: per-joint step + current cmd
  step_per_joint_.assign(joint_count_, 0.2);
  for (int i = 0; i < joint_count_; ++i) {
    if (i <= 2) step_per_joint_[i] = 0.05;  // Joint_0..2 default
  }
  current_cmd_.assign(joint_count_, 0.0);
  rebuildRows();

  // ROS + publisher
  ensureNode();
  rebuildPublisher();

  // Publisher timer
  connect(&publish_timer_, &QTimer::timeout, this, &JointJoggerPanel::publishOnce);
  onRateChanged(rate_hz_);
  publish_timer_.start();
}

void JointJoggerPanel::ensureNode()
{
  if (!node_) {
    node_ = std::make_shared<rclcpp::Node>("joint_jogger_panel");
  }
}

void JointJoggerPanel::rebuildPublisher()
{
  ensureNode();
  pub_.reset();
  pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_, rclcpp::QoS(10));
}

void JointJoggerPanel::addJointRow(int index)
{
  const int row = index;

  auto * name_label = new QLabel(QString("Joint_%1").arg(index), joints_container_);
  auto * minus_btn = new QToolButton(joints_container_);
  auto * plus_btn = new QToolButton(joints_container_);
  auto * step_box = new QDoubleSpinBox(joints_container_);

  // Arrow buttons
  minus_btn->setArrowType(Qt::LeftArrow);
  plus_btn->setArrowType(Qt::RightArrow);
  minus_btn->setToolButtonStyle(Qt::ToolButtonIconOnly);
  plus_btn->setToolButtonStyle(Qt::ToolButtonIconOnly);
  minus_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  plus_btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  minus_btn->setIconSize(QSize(28, 28));
  plus_btn->setIconSize(QSize(28, 28));
  minus_btn->setAutoRepeat(true);
  plus_btn->setAutoRepeat(true);

  // Per-joint step
  step_box->setRange(0.001, 10.0);
  step_box->setDecimals(3);
  step_box->setValue(step_per_joint_.at(static_cast<size_t>(index)));
  QObject::connect(
    step_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this, index](double v) {
      if (index >= 0 && index < joint_count_) step_per_joint_[static_cast<size_t>(index)] = v;
    });

  // Jog connections using per-joint step
  QObject::connect(minus_btn, &QToolButton::pressed, [this, index]() {
    setVelocityForIndex(static_cast<size_t>(index), -step_per_joint_[static_cast<size_t>(index)]);
  });
  QObject::connect(plus_btn, &QToolButton::pressed, [this, index]() {
    setVelocityForIndex(static_cast<size_t>(index), +step_per_joint_[static_cast<size_t>(index)]);
  });
  QObject::connect(minus_btn, &QToolButton::released, [this, index]() {
    setVelocityForIndex(static_cast<size_t>(index), 0.0);
  });
  QObject::connect(plus_btn, &QToolButton::released, [this, index]() {
    setVelocityForIndex(static_cast<size_t>(index), 0.0);
  });

  joints_grid_->addWidget(name_label, row, 0);
  joints_grid_->addWidget(minus_btn, row, 1);
  joints_grid_->addWidget(plus_btn, row, 2);
  joints_grid_->addWidget(step_box, row, 3);
}

void JointJoggerPanel::rebuildRows()
{
  // Clear grid
  QLayoutItem * child;
  while ((child = joints_grid_->takeAt(0)) != nullptr) {
    if (child->widget()) child->widget()->deleteLater();
    delete child;
  }
  // Recreate rows
  for (int i = 0; i < joint_count_; ++i) addJointRow(i);
}

void JointJoggerPanel::setVelocityForIndex(size_t idx, double vel)
{
  if (idx >= current_cmd_.size()) return;
  current_cmd_[idx] = vel;
}

void JointJoggerPanel::publishOnce()
{
  if (!publishing_enabled_) return;
  if (!pub_) return;

  std_msgs::msg::Float64MultiArray msg;
  msg.data = current_cmd_; // exactly N elements
  pub_->publish(msg);
}

void JointJoggerPanel::onTopicEdited()
{
  topic_ = topic_edit_->text().toStdString();
  rebuildPublisher();
}

void JointJoggerPanel::onRateChanged(double hz)
{
  rate_hz_ = hz;
  int ms = std::max(1, static_cast<int>(1000.0 / rate_hz_));
  publish_timer_.setInterval(ms);
}

void JointJoggerPanel::onStopAll()
{
  std::fill(current_cmd_.begin(), current_cmd_.end(), 0.0);
  publishOnce();
}

void JointJoggerPanel::onCountChanged(int n)
{
  if (n < 1) n = 1;
  joint_count_ = n;

  const size_t old = step_per_joint_.size();
  step_per_joint_.resize(joint_count_, 0.2);
  for (size_t i = old; i < step_per_joint_.size(); ++i) {
    if (i <= 2) step_per_joint_[i] = 0.05;
  }
  current_cmd_.assign(joint_count_, 0.0);

  rebuildRows();
}

void JointJoggerPanel::onAddOne() { count_spin_->setValue(count_spin_->value() + 1); }

void JointJoggerPanel::onRemoveOne()
{
  count_spin_->setValue(std::max(1, count_spin_->value() - 1));
}

void JointJoggerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString qtopic;
  if (config.mapGetString("topic", &qtopic)) {
    topic_ = qtopic.toStdString();
    if (topic_edit_) topic_edit_->setText(qtopic);
    rebuildPublisher();
  }
  float rate_tmp;
  if (config.mapGetFloat("rate_hz", &rate_tmp)) {
    rate_hz_ = rate_tmp;
    if (rate_spin_) rate_spin_->setValue(rate_hz_);
    onRateChanged(rate_hz_);
  }
  int jc_tmp;
  if (config.mapGetInt("joint_count", &jc_tmp)) {
    count_spin_->setValue(std::max(1, jc_tmp));
  }
}

void JointJoggerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("topic", QString::fromStdString(topic_));
  config.mapSetValue("rate_hz", static_cast<float>(rate_hz_));
  config.mapSetValue("joint_count", joint_count_);
}

void JointJoggerPanel::onTogglePublishing()
{
  publishing_enabled_ = !publishing_enabled_;  // flip state

  if (publishing_enabled_) {
    // Turn ON
    toggle_btn_->setText("ON");
    toggle_btn_->setStyleSheet("background-color: green; color: white; font-weight: bold;");

    if (!pub_) {
      rebuildPublisher();
    }

    onRateChanged(rate_hz_);
    if (!publish_timer_.isActive()) {
      publish_timer_.start();
    }
  } else {
    // Turn OFF
    toggle_btn_->setText("OFF");
    toggle_btn_->setStyleSheet("background-color: red; color: white; font-weight: bold;");

    // Send a final zero command
    if (pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(static_cast<size_t>(joint_count_), 0.0);
      pub_->publish(msg);
    }

    publish_timer_.stop();
    pub_.reset();
  }
}

// Export plugin
}  // namespace joint_jogger_gui
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joint_jogger_gui::JointJoggerPanel, rviz_common::Panel)
