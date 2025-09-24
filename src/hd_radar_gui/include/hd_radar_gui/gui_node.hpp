#ifndef HD_RADAR_GUI_GUI_NODE_HPP
#define HD_RADAR_GUI_GUI_NODE_HPP

#include <QWidget>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <hd_radar_interfaces/srv/set_thr.hpp>
#include <hd_radar_interfaces/srv/set_mode.hpp>

class RadarGuiWidget : public QWidget {
  Q_OBJECT
public:
  explicit RadarGuiWidget(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);

private slots:
  void onSendThresholds();
  void onSendMode();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<hd_radar_interfaces::srv::SetThr>::SharedPtr thr_client_;
  rclcpp::Client<hd_radar_interfaces::srv::SetMode>::SharedPtr mode_client_;

  QLineEdit * service_prefix_edit_;

  QSpinBox * sta_threshold_;
  QSpinBox * sta_azm_sense_;
  QSpinBox * sta_rcs_filter_;
  QSpinBox * dyn_threshold_;
  QSpinBox * dyn_azm_sense_;
  QSpinBox * dyn_rcs_filter_;

  QSpinBox * mode_;

  QPushButton * send_thr_btn_;
  QPushButton * send_mode_btn_;

  void updateServiceClients();
};

#endif


