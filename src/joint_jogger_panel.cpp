#include "joint_jogger_gui/joint_jogger_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSpacerItem>

#include <algorithm>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

namespace joint_jogger_gui {

JointJoggerPanel::JointJoggerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout(this);

  // Controls box
  auto * ctrl_box = new QGroupBox("Settings", this);
  auto * ctrl_layout = new QGridLayout(ctrl_box);

  topic_edit_ = new QLineEdit(QString::fromStdString(topic_), ctrl_box);
  connect(topic_edit_, &QLineEdit::editingFinished, this, &JointJoggerPanel::onTopicEdited);

  rate_spin_ = new QDoubleSpinBox(ctrl_box);
  rate_spin_->setRange(1.0, 200.0);
  rate_spin_->setDecimals(1);
  rate_spin_->setValue(rate_hz_);
  connect(rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &JointJoggerPanel::onRateChanged);

  step_spin_ = new QDoubleSpinBox(ctrl_box);
  step_spin_->setRange(0.01, 10.0);
  step_spin_->setDecimals(3);
  step_spin_->setValue(step_);
  connect(step_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &JointJoggerPanel::onStepChanged);

  add_joint_edit_ = new QLineEdit(ctrl_box);
  add_joint_btn_ = new QPushButton("Add joint", ctrl_box);
  connect(add_joint_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onAddJoint);

  clear_btn_ = new QPushButton("Clear", ctrl_box);
  connect(clear_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onClearAll);

  stop_all_btn_ = new QPushButton("Stop All", ctrl_box);
  connect(stop_all_btn_, &QPushButton::clicked, this, &JointJoggerPanel::onStopAll);

  int r = 0;
  ctrl_layout->addWidget(new QLabel("Topic:"), r, 0);
  ctrl_layout->addWidget(topic_edit_, r, 1, 1, 3); r++;
  ctrl_layout->addWidget(new QLabel("Rate (Hz):"), r, 0);
  ctrl_layout->addWidget(rate_spin_, r, 1);
  ctrl_layout->addWidget(new QLabel("Step vel:"), r, 2);
  ctrl_layout->addWidget(step_spin_, r, 3); r++;
  ctrl_layout->addWidget(new QLabel("Joint name:"), r, 0);
  ctrl_layout->addWidget(add_joint_edit_, r, 1);
  ctrl_layout->addWidget(add_joint_btn_, r, 2);
  ctrl_layout->addWidget(clear_btn_, r, 3); r++;
  ctrl_layout->addWidget(stop_all_btn_, r, 0, 1, 4); r++;

  ctrl_box->setLayout(ctrl_layout);
  root_layout->addWidget(ctrl_box);

  // Joints scroll area
  joints_container_ = new QWidget(this);
  joints_grid_ = new QGridLayout(joints_container_);
  joints_grid_->setColumnStretch(2, 1);

  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(joints_container_);

  root_layout->addWidget(scroll, 1);

  // Timer for publishing
  publish_timer_.setInterval(static_cast<int>(1000.0 / rate_hz_));
  connect(&publish_timer_, &QTimer::timeout, this, &JointJoggerPanel::publishOnce);
  publish_timer_.start();

  ensureNode();
  rebuildPublisher();
}

void JointJoggerPanel::ensureNode() {
  if (!node_) {
    node_ = std::make_shared<rclcpp::Node>("joint_jogger_panel");
  }
}

void JointJoggerPanel::rebuildPublisher() {
  ensureNode();
  pub_.reset();
  pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic_, rclcpp::QoS(10));
}

void JointJoggerPanel::addJointRow(const std::string & joint) {
  if (rows_.count(joint)) return;
  int row = static_cast<int>(rows_.size());

  auto * name_label = new QLabel(QString::fromStdString(joint), joints_container_);
  auto * minus_btn = new QPushButton("–", joints_container_);
  auto * plus_btn  = new QPushButton("+", joints_container_);

  minus_btn->setAutoRepeat(true);
  plus_btn->setAutoRepeat(true);

  // Expand vectors
  joint_names_.push_back(joint);
  current_cmd_.resize(joint_names_.size(), 0.0);

  // Connections: press sets a velocity; release sets zero
  connect(minus_btn, &QPushButton::pressed, [this, joint]() { setVelocityFor(joint, -step_); });
  connect(plus_btn,  &QPushButton::pressed, [this, joint]() { setVelocityFor(joint, +step_); });
  connect(minus_btn, &QPushButton::released, [this, joint]() { setVelocityFor(joint, 0.0); });
  connect(plus_btn,  &QPushButton::released, [this, joint]() { setVelocityFor(joint, 0.0); });

  joints_grid_->addWidget(name_label, row, 0);
  joints_grid_->addWidget(minus_btn, row, 1);
  joints_grid_->addWidget(plus_btn,  row, 2);

  rows_[joint] = {name_label, minus_btn, plus_btn};
}

void JointJoggerPanel::setVelocityFor(const std::string & joint, double vel) {
  auto it = std::find(joint_names_.begin(), joint_names_.end(), joint);
  if (it == joint_names_.end()) return;
  size_t idx = static_cast<size_t>(std::distance(joint_names_.begin(), it));
  if (idx >= current_cmd_.size()) current_cmd_.resize(joint_names_.size(), 0.0);
  current_cmd_[idx] = vel;
}

void JointJoggerPanel::publishOnce() {
  if (!pub_) return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data = current_cmd_;
  pub_->publish(msg);
}

void JointJoggerPanel::onTopicEdited() {
  topic_ = topic_edit_->text().toStdString();
  rebuildPublisher();
}

void JointJoggerPanel::onRateChanged(double hz) {
  rate_hz_ = hz;
  int ms = std::max(1, static_cast<int>(1000.0 / rate_hz_));
  publish_timer_.setInterval(ms);
}

void JointJoggerPanel::onStepChanged(double step) {
  step_ = step;
}

void JointJoggerPanel::onAddJoint() {
  auto name = add_joint_edit_->text().trimmed().toStdString();
  if (name.empty()) return;
  addJointRow(name);
  add_joint_edit_->clear();
}

void JointJoggerPanel::onClearAll() {
  // Reset data
  joint_names_.clear();
  current_cmd_.clear();
  // Remove widgets
  QLayoutItem * child;
  while ((child = joints_grid_->takeAt(0)) != nullptr) {
    if (child->widget()) child->widget()->deleteLater();
    delete child;
  }
  rows_.clear();
}

void JointJoggerPanel::onStopAll() {
  std::fill(current_cmd_.begin(), current_cmd_.end(), 0.0);
  publishOnce();
}

void JointJoggerPanel::load(const rviz_common::Config & config) {
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
  float step_tmp;
  if (config.mapGetFloat("step", &step_tmp)) {
    step_ = step_tmp;
    if (step_spin_) step_spin_->setValue(step_);
  }

  // Load joints as a semicolon-separated list
  QString qjoints;
  if (config.mapGetString("joints", &qjoints)) {
    for (const auto & part : qjoints.split(";", Qt::SkipEmptyParts)) {
      addJointRow(part.toStdString());
    }
  }
}

void JointJoggerPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("topic", QString::fromStdString(topic_));
  config.mapSetValue("rate_hz", static_cast<float>(rate_hz_));
  config.mapSetValue("step", static_cast<float>(step_));

  QString qjoints;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    qjoints += QString::fromStdString(joint_names_[i]);
    if (i + 1 < joint_names_.size()) qjoints += ";";
  }
  config.mapSetValue("joints", qjoints);
}

} // namespace joint_jogger_gui
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joint_jogger_gui::JointJoggerPanel, rviz_common::Panel)
