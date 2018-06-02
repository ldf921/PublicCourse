// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/display/main_window.h"
#include "pnc/simulation/replay_system.h"

namespace display {

class ReplayMainWindow : public MainWindow {
 public:
  ReplayMainWindow(QWidget* parent, const std::string& simulation_history_path)
      : MainWindow(interface::simulation::SimulationConfig(), parent) {
    setWindowTitle("Replay");
    replay_system_ = std::make_unique<simulation::ReplaySystem>(simulation_history_path);
    replay_system_thread_ = std::thread([this]() {
      replay_system_->Initialize();
      replay_system_->Start();
    });
    SetupMenu();
  }

  void timerEvent(QTimerEvent* /*event*/) override {
    data_ = replay_system_->FetchData();
    if (!perspective_menu_setup_) {
      SetupPerspectiveMenu();
      perspective_menu_setup_ = true;
    }
    painter_widget_->set_simulation_system_data(data_);
    painter_widget_->update();
    std::unordered_map<std::string, interface::display::VariableViewList> variable_view_data;
    GetVariableViewList(&variable_view_data, data_);
    variable_dock_widget_->UpdateVariables(variable_view_data);
  }

 private:
  void SetupMenu() {
    QAction* action = nullptr;
    QMenu* menu = nullptr;
    // Play
    menu = menuBar()->addMenu(tr("&Play"));

    action = menu->addAction(tr("&Pause/Resume"));
    action->setShortcut(QKeySequence(Qt::Key_Space));
    connect(action, &QAction::triggered, this, [this] { replay_system_->PushPauseCommand(); });

    menu->addSeparator();

    action = menu->addAction(tr("&Half speed"));
    QList<QKeySequence> shortcuts;
    shortcuts << QKeySequence(Qt::Key_Minus) << QKeySequence(Qt::Key_Underscore);
    action->setShortcuts(shortcuts);
    connect(action, &QAction::triggered, this,
            [this] { replay_system_->PushSimulationSpeedCommand(0.5); });

    action = menu->addAction(tr("&Double speed"));
    shortcuts.clear();
    shortcuts << QKeySequence(Qt::Key_Plus) << QKeySequence(Qt::Key_Equal);
    action->setShortcuts(shortcuts);
    connect(action, &QAction::triggered, this,
            [this] { replay_system_->PushSimulationSpeedCommand(2.0); });
    menu->addSeparator();
    action = menu->addAction(tr("&Quit"));
    action->setShortcut(QKeySequence::Quit);
    connect(action, &QAction::triggered, this, [this] { QApplication::exit(0); });
  }

  void SetupPerspectiveMenu() {
    QAction* action = nullptr;
    QMenu* menu = nullptr;
    // Play
    menu = menuBar()->addMenu(tr("&Perspective"));

    action = menu->addAction(tr("&God"));
    connect(action, &QAction::triggered, this,
            [this] { painter_widget_->SetVehiclePerspective(""); });

    menu->addSeparator();

    for (const auto& agent_data : data_.vehicle_agent()) {
      vehicle_name_list_.push_back(agent_data.name());
    }

    for (const std::string& name : vehicle_name_list_) {
      action = menu->addAction(tr(name.data()));
      connect(action, &QAction::triggered, this,
              [this, &name] {
                painter_widget_->SetVehiclePerspective(name); });
    }
  }

  std::unique_ptr<simulation::ReplaySystem> replay_system_;
  std::thread replay_system_thread_;

  interface::simulation::SimulationSystemData data_;

  std::vector<std::string> vehicle_name_list_;
  bool perspective_menu_setup_ = false;

};
};
