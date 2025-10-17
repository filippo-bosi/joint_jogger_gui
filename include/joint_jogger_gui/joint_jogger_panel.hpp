#pragma once

#include <QPushButton>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QTimer>
#include <QWidget>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace joint_jogger_gui {

class JointJoggerPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit JointJoggerPanel(QWidget * parent = nullptr);
  ~JointJoggerPanel() override = default;

  // Save/restore RViz config
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onTopicEdited();
  void onRateChanged(double hz);
  void onStepChanged(double step);
  void onAddJoint();
  void onClearAll();
  void onStopAll();

private:
  struct RowWidgets {
    QLabel * name_label {nullptr};
    QPushButton * minus_btn {nullptr};
    QPushButton * plus_btn {nullptr};
  };

  void ensureNode();
  void rebuildPublisher();
  void publishOnce();
  void setVelocityFor(const std::string & joint, double vel);
  void addJointRow(const std::string & joint);

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  // GUI widgets
  QLineEdit * topic_edit_ {nullptr};
  QDoubleSpinBox * rate_spin_ {nullptr};
  QDoubleSpinBox * step_spin_ {nullptr};
  QLineEdit * add_joint_edit_ {nullptr};
  QPushButton * add_joint_btn_ {nullptr};
  QPushButton * clear_btn_ {nullptr};
  QPushButton * stop_all_btn_ {nullptr};

  QWidget * joints_container_ {nullptr};
  QGridLayout * joints_grid_ {nullptr};
  QTimer publish_timer_;

  // Data
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, RowWidgets> rows_;
  std::vector<double> current_cmd_;

  // Defaults
  std::string topic_ {"/velocity_controller/commands"};
  double rate_hz_ {50.0};
  double step_ {0.2}; // rad/s by default
};

} // namespace joint_jogger_gui