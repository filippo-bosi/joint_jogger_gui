#pragma once

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QWidget>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>

namespace joint_jogger_gui
{

class JointJoggerPanel : public rviz_common::Panel
{
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
  void onStopAll();
  void onCountChanged(int n);
  void onAddOne();
  void onRemoveOne();
  void onTogglePublishing();

private:
  struct RowWidgets
  {
    QLabel * name_label{nullptr};
    QToolButton * minus_btn{nullptr};
    QToolButton * plus_btn{nullptr};
    QDoubleSpinBox * step_spin{nullptr};
  };

  void ensureNode();
  void rebuildPublisher();
  void publishOnce();
  void setVelocityForIndex(size_t idx, double vel);
  void addJointRow(int index);
  void rebuildRows();

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  // GUI widgets
  QLineEdit * topic_edit_{nullptr};
  QDoubleSpinBox * rate_spin_{nullptr};
  QSpinBox * count_spin_{nullptr};
  QPushButton * add_one_btn_{nullptr};
  QPushButton * remove_one_btn_{nullptr};
  QPushButton * stop_all_btn_{nullptr};
  QPushButton * toggle_btn_{nullptr};

  QWidget * joints_container_{nullptr};
  QGridLayout * joints_grid_{nullptr};
  QTimer publish_timer_;

  // Data
  std::vector<double> current_cmd_;
  std::vector<double> step_per_joint_;


  // Defaults
  int joint_count_{7};
  std::string topic_{"/velocity_controller/commands"};
  double rate_hz_{50.0};
  bool publishing_enabled_{false};
};

}  // namespace joint_jogger_gui
