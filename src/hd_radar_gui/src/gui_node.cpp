#include "hd_radar_gui/gui_node.hpp"

#include <QMessageBox>

using SetThr = hd_radar_interfaces::srv::SetThr;
using SetMode = hd_radar_interfaces::srv::SetMode;

RadarGuiWidget::RadarGuiWidget(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QWidget(parent), node_(std::move(node))
{
  auto * main_layout = new QVBoxLayout(this);

  service_prefix_edit_ = new QLineEdit(this);
  service_prefix_edit_->setPlaceholderText("frame_id (e.g. hd_radar)");
  service_prefix_edit_->setText("hd_radar");

  auto * prefix_layout = new QHBoxLayout();
  prefix_layout->addWidget(new QLabel("Frame ID:", this));
  prefix_layout->addWidget(service_prefix_edit_);
  main_layout->addLayout(prefix_layout);

  auto * form = new QFormLayout();

  sta_threshold_ = new QSpinBox(this);
  sta_threshold_->setRange(0, 1000);
  sta_threshold_->setValue(13);
  form->addRow("Static threshold", sta_threshold_);

  sta_azm_sense_ = new QSpinBox(this);
  sta_azm_sense_->setRange(0, 1000);
  sta_azm_sense_->setValue(16);
  form->addRow("Static azm sense", sta_azm_sense_);

  sta_rcs_filter_ = new QSpinBox(this);
  sta_rcs_filter_->setRange(-200, 200);
  sta_rcs_filter_->setValue(-48);
  form->addRow("Static rcs filter", sta_rcs_filter_);

  dyn_threshold_ = new QSpinBox(this);
  dyn_threshold_->setRange(0, 1000);
  dyn_threshold_->setValue(13);
  form->addRow("Dynamic threshold", dyn_threshold_);

  dyn_azm_sense_ = new QSpinBox(this);
  dyn_azm_sense_->setRange(0, 1000);
  dyn_azm_sense_->setValue(16);
  form->addRow("Dynamic azm sense", dyn_azm_sense_);

  dyn_rcs_filter_ = new QSpinBox(this);
  dyn_rcs_filter_->setRange(-200, 200);
  dyn_rcs_filter_->setValue(-48);
  form->addRow("Dynamic rcs filter", dyn_rcs_filter_);

  main_layout->addLayout(form);

  send_thr_btn_ = new QPushButton("Send thresholds", this);
  connect(send_thr_btn_, &QPushButton::clicked, this, &RadarGuiWidget::onSendThresholds);
  main_layout->addWidget(send_thr_btn_);

  auto * mode_layout = new QHBoxLayout();
  mode_ = new QSpinBox(this);
  mode_->setRange(0, 255);
  mode_->setValue(0);
  mode_layout->addWidget(new QLabel("Mode:", this));
  mode_layout->addWidget(mode_);
  send_mode_btn_ = new QPushButton("Send mode", this);
  connect(send_mode_btn_, &QPushButton::clicked, this, &RadarGuiWidget::onSendMode);
  mode_layout->addWidget(send_mode_btn_);
  main_layout->addLayout(mode_layout);

  updateServiceClients();
  connect(service_prefix_edit_, &QLineEdit::editingFinished, this, &RadarGuiWidget::updateServiceClients);
}

void RadarGuiWidget::updateServiceClients()
{
  const auto prefix = service_prefix_edit_->text().toStdString();
  const auto thr_name = "/" + prefix + std::string("_set_thr");
  const auto mode_name = "/" + prefix + std::string("_set_mode");
  thr_client_ = node_->create_client<SetThr>(thr_name);
  mode_client_ = node_->create_client<SetMode>(mode_name);
}

void RadarGuiWidget::onSendThresholds()
{
  if (!thr_client_) return;
  if (!thr_client_->service_is_ready()) {
    QMessageBox::warning(this, "Service", "SetThr service is not available");
    return;
  }

  auto req = std::make_shared<SetThr::Request>();
  req->sta_threshold = static_cast<uint16_t>(sta_threshold_->value());
  req->sta_azm_sense = static_cast<uint16_t>(sta_azm_sense_->value());
  req->sta_rcs_filter = static_cast<int16_t>(sta_rcs_filter_->value());
  req->dyn_threshold = static_cast<uint16_t>(dyn_threshold_->value());
  req->dyn_azm_sense = static_cast<uint16_t>(dyn_azm_sense_->value());
  req->dyn_rcs_filter = static_cast<int16_t>(dyn_rcs_filter_->value());

  auto future = thr_client_->async_send_request(req);
  // Optionally wait briefly and show result
}

void RadarGuiWidget::onSendMode()
{
  if (!mode_client_) return;
  if (!mode_client_->service_is_ready()) {
    QMessageBox::warning(this, "Service", "SetMode service is not available");
    return;
  }

  auto req = std::make_shared<SetMode::Request>();
  req->mode = static_cast<uint8_t>(mode_->value());
  auto future = mode_client_->async_send_request(req);
}


