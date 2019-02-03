#ifndef SIMCONTROLPANEL_H
#define SIMCONTROLPANEL_H

#include <QMainWindow>
#include <thread>
#include "Simulation.h"
#include "Graphics3D.h"
#include "ControlParameters/SimulatorParameters.h"

namespace Ui {
  class SimControlPanel;
}

class SimControlPanel : public QMainWindow {
Q_OBJECT

public:
  explicit SimControlPanel(QWidget *parent = nullptr);

  ~SimControlPanel();

private slots:

  void on_startButton_clicked();

  void on_stopButton_clicked();

  void on_joystickButton_clicked();

  void on_driverButton_clicked();

  void on_simulatorTable_cellChanged(int row, int column);

  void on_saveSimulatorButton_clicked();

  void on_loadSimulatorButton_clicked();

  void on_robotTable_cellChanged(int row, int column);

  void on_saveRobotButton_clicked();

  void on_loadRobotButton_clicked();

  void on_favoritesTable_cellChanged(int row, int column);

  void on_loadFavoriteButton_clicked();

  void loadSimulationParameters(SimulatorControlParameters& params);
  void loadRobotParameters(RobotControlParameters& params);

private:
  void updateUiEnable();
  std::thread _simThread;
  bool _started = false;
  Ui::SimControlPanel *ui;
  Simulation *_simulation = nullptr;
  Graphics3D *_graphicsWindow = nullptr;
  SimulatorControlParameters _parameters;
  bool _simulationMode = false;
  bool _firstStart = true;
  bool _ignoreTableCallbacks = false;
};

#endif // SIMCONTROLPANEL_H
