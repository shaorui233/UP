#include "SimControlPanel.h"
#include <QMessageBox>
#include <QFileDialog>
#include <ControlParameters/ControlParameters.h>
#include "ui_SimControlPanel.h"

/*!
 * Display an error messagebox with the given text
 */
static void createErrorMessage(const std::string& text) {
  QMessageBox mb;
  mb.setText(QString(text.c_str()));
  mb.exec();
}


SimControlPanel::SimControlPanel(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimControlPanel) {
  ui->setupUi(this);
  updateUiEnable();
  _parameters.initializeFromIniFile(getConfigDirectoryPath() + SIMULATOR_DEFAULT_PARAMETERS);
  loadSimulationParameters(_parameters);
}

SimControlPanel::~SimControlPanel() {
  delete _simulation;
  delete _graphicsWindow;
  delete ui;
}

/*!
 * Enable/disable buttons as needed based on what is running
 */
void SimControlPanel::updateUiEnable() {
  ui->startButton->setEnabled(!_started);
  ui->stopButton->setEnabled(_started);
  ui->joystickButton->setEnabled(_started);
//  ui->saveSimulatorButton->setEnabled(_started && _simulationMode);
//  ui->simulatorTable->setEnabled(_started && _simulationMode);
  ui->robotTable->setEnabled(_started);
}

void SimControlPanel::on_startButton_clicked() {
  RobotType robotType;

  if(ui->cheetah3Button->isChecked()) {
    robotType = RobotType::CHEETAH_3;
  } else if(ui->miniCheetahButton->isChecked()) {
    robotType = RobotType::MINI_CHEETAH;
  } else {
    createErrorMessage("Error: you must select a robot");
    return;
  }

  if(!ui->simulatorButton->isChecked() && !ui->robotButton->isChecked()) {
    createErrorMessage("Error: you must select either robot or simulation mode");
    return;
  }

  _simulationMode = ui->simulatorButton->isChecked();

  if(_simulationMode) {
    _graphicsWindow = new Graphics3D();
    _graphicsWindow->show();
    _graphicsWindow->resize(1280, 720);
    _simulation = new Simulation(robotType, _graphicsWindow, _parameters);
    loadSimulationParameters(_simulation->getSimParams());
    loadRobotParameters(_simulation->getRobotParams());

    // hack
    _simulation->addCollisionPlane(.8, 0, -0.5);
//    // Box 1
//    Vec3<double> pos;
//    Mat3<double> ori;
//    Vec3<double> ori_zyx; ori_zyx.setZero();
//
//    pos[0] = 0.0; pos[1] = 0.0; pos[2] = -0.415;
//
//    ori_zyx[0] = 0.3;
//    ori_zyx[1] = 0.4;
//    EulerZYX_2_SO3(ori_zyx, ori);
//    _simulation->addCollisionBox(0.8, 0., 1.5, 0.7, 0.05, pos, ori);
//
//    // Box 2
//    pos[0] = 0.3; pos[1] = 0.05;  pos[2] = -0.30;
//    ori_zyx[0] = 0.4; ori_zyx[1] = 0.;  ori_zyx[2] = 0.4;
//    EulerZYX_2_SO3(ori_zyx, ori);
//    _simulation->addCollisionBox(0.8, 0., 0.7, 0.7, 0.05, pos, ori);

    //
    _simThread = std::thread([this](){_simulation->runAtSpeed();});

    _graphicsWindow->setAnimating(true);
  } else {
    assert(false); // don't support robot mode yet
  }

  _started = true;
  updateUiEnable();
}

void SimControlPanel::on_stopButton_clicked() {
  if(_simulation) {
    _simulation->stop();
    _simThread.join();
  }

  if(_graphicsWindow) {
    _graphicsWindow->setAnimating(false);
    _graphicsWindow->hide();
  }

  delete _simulation;
  delete _graphicsWindow;

  _simulation = nullptr;
  _graphicsWindow = nullptr;

  _started = false;
  updateUiEnable();
}

static void updateQtableWithParameters(ControlParameters& params, QTableWidget& table) {
  table.setRowCount((s32)params.collection._map.size());

  s32 i = 0;
  for(auto& kv : params.collection._map) {
    for(s32 col = 0; col < 2; col++) {
      QTableWidgetItem* cell = table.item(i,col);
      if(!cell) {
        cell = new QTableWidgetItem;
        table.setItem(i,col,cell);
      }
    }

    table.item(i,0)->setText(QString(kv.first.c_str()));
    table.item(i,1)->setText(QString(kv.second->toString().c_str()));
    i++;
  }
}

/*!
 * Reload all values in the Simulation Parameter Table
 */
void SimControlPanel::loadSimulationParameters(SimulatorControlParameters &params) {
  _ignoreTableCallbacks = true;
  updateQtableWithParameters(params, *ui->simulatorTable);
  _ignoreTableCallbacks = false;
}

void SimControlPanel::loadRobotParameters(RobotControlParameters &params) {
  _ignoreTableCallbacks = true;
  updateQtableWithParameters(params, *ui->robotTable);
  _ignoreTableCallbacks = false;
}

void SimControlPanel::on_joystickButton_clicked() {
  _graphicsWindow->resetGameController();
}

void SimControlPanel::on_driverButton_clicked() {

}

void SimControlPanel::on_simulatorTable_cellChanged(int row, int column) {
  if(_ignoreTableCallbacks) return;


  if(column != 1) {
    return;
  }

  auto cell = ui->simulatorTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if(cellName == "") {
    return;
  }

  printf("cell %s changed to %s\n", cellName.c_str(), ui->simulatorTable->item(row, 1)->text().toStdString().c_str());

  auto& parameter = _parameters.collection.lookup(cellName);
  parameter.setFromString(ui->simulatorTable->item(row, 1)->text().toStdString());
}

void SimControlPanel::on_saveSimulatorButton_clicked() {
  QString fileName = QFileDialog::getSaveFileName(nullptr, ("Save Simulator Table Values"), "../config", "All Files (*)");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }
  _simulation->getSimParams().lockMutex();
  _simulation->getSimParams().writeToIniFile(fileName.toStdString());
  _simulation->getSimParams().unlockMutex();
}

void SimControlPanel::on_loadSimulatorButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(nullptr, ("Save Simulator Table Values"), "../config", "All Files (*)");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  _simulation->getSimParams().lockMutex();
  _simulation->getSimParams().collection.clearAllSet();
  _simulation->getSimParams().initializeFromIniFile(fileName.toStdString());
  if(!_simulation->getSimParams().collection.checkIfAllSet()) {
    printf("new settings file %s doesn't contain the following simulator parameters:\n%s\n",
            fileName.toStdString().c_str(), _simulation->getSimParams().generateUnitializedList().c_str());
    throw std::runtime_error("bad new settings file");
  }
  loadSimulationParameters(_simulation->getSimParams());
  _simulation->getSimParams().unlockMutex();
}

void SimControlPanel::on_robotTable_cellChanged(int row, int column) {
  if(_ignoreTableCallbacks) return;
  if(column != 1) {
    return;
  }

  auto cell = ui->robotTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if(cellName == "") {
    return;
  }

  printf("cell %s changed to %s\n", cellName.c_str(), ui->robotTable->item(row, 1)->text().toStdString().c_str());

  auto& parameter = _simulation->getRobotParams().collection.lookup(cellName);
  parameter.setFromString(ui->robotTable->item(row, 1)->text().toStdString());

  if(_simulationMode) {
    if(_simulation->isRobotConnected()) {
      _simulation->sendControlParameter(cellName, parameter.get(parameter._kind), parameter._kind);
    }
  } else {
    assert(false);
  }

}

void SimControlPanel::on_saveRobotButton_clicked() {
  printf("save callback\n");
  QString fileName = QFileDialog::getSaveFileName(nullptr, ("Save Quadruped Table Values"), "../config", "All Files (*)");
  printf("2\n");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }
  printf("3\n");
  _simulation->getRobotParams().writeToIniFile(fileName.toStdString());
}

void SimControlPanel::on_loadRobotButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(nullptr, ("Save Quadruped Table Values"), "../config", "All Files (*)");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  if(_simulationMode) {
    _simulation->getRobotParams().lockMutex();
    _simulation->getRobotParams().collection.clearAllSet();
    _simulation->getRobotParams().initializeFromIniFile(fileName.toStdString());
    if(!_simulation->getRobotParams().collection.checkIfAllSet()) {
      printf("new settings file %s doesn't contain the following simulator parameters:\n%s\n",
             fileName.toStdString().c_str(), _simulation->getRobotParams().generateUnitializedList().c_str());
      throw std::runtime_error("bad new settings file");
    }
    loadRobotParameters(_simulation->getRobotParams());

    if(_simulation->isRobotConnected()) {
      for(auto& kv : _simulation->getRobotParams().collection._map) {
        _simulation->sendControlParameter(kv.first, kv.second->get(kv.second->_kind), kv.second->_kind);
      }
    }
    _simulation->getSimParams().unlockMutex();
  } else {
    assert(false);
  }
}

void SimControlPanel::on_favoritesTable_cellChanged(int row, int column) {
  (void) row;
  (void) column;
}

void SimControlPanel::on_loadFavoriteButton_clicked() {

}
