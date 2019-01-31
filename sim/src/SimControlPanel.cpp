#include "SimControlPanel.h"
#include <QMessageBox>
#include <QFileDialog>
#include <ControlParameters.h>
#include "ui_SimControlPanel.h"
#include "../../build/sim/ui_SimControlPanel.h" // todo remove me!

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
  ui->saveSimulatorButton->setEnabled(_started && _simulationMode);
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
    _simulation = new Simulation(robotType, _graphicsWindow);
    loadSimulationParameters(_simulation->getParams());

    // hack
    _simulation->addCollisionPlane(.8, 0, -0.5);
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

/*!
 * Reload all values in the Simulation Parameter Table
 */
void SimControlPanel::loadSimulationParameters(SimulatorControlParameters &params) {
  ui->simulatorTable->setRowCount((s32)params.collection._map.size());

  s32 i = 0;
  for(auto& kv : params.collection._map) {
    for(s32 col = 0; col < 2; col++) {
      QTableWidgetItem* cell = ui->simulatorTable->item(i,col);
      if(!cell) {
        cell = new QTableWidgetItem;
        ui->simulatorTable->setItem(i,col,cell);
      }
    }

    ui->simulatorTable->item(i,0)->setText(QString(kv.first.c_str()));
    ui->simulatorTable->item(i,1)->setText(QString(kv.second->toString().c_str()));
    i++;
  }
}

void SimControlPanel::on_joystickButton_clicked() {
  _graphicsWindow->resetGameController();
}

void SimControlPanel::on_driverButton_clicked() {

}

void SimControlPanel::on_simulatorTable_cellChanged(int row, int column) {
  if(!_simulation) {
    createErrorMessage("can't do that now");
    return;
  }

  if(column != 1) {
    return;
  }


  auto cell = ui->simulatorTable->item(row, 0);
  std::string cellName = cell->text().toStdString();

  if(cellName == "") {
    return;
  }

  printf("cell %s changed to %s\n", cellName.c_str(), ui->simulatorTable->item(row, 1)->text().toStdString().c_str());

  auto& parameter = _simulation->getParams().collection.lookup(cellName);
  parameter.setFromString(ui->simulatorTable->item(row, 1)->text().toStdString());
//
//  loadSimulationParameters(_simulation->getParams());
}

void SimControlPanel::on_saveSimulatorButton_clicked() {
  QString fileName = QFileDialog::getSaveFileName(nullptr, ("Save Simulator Table Values"), "../config", "All Files (*)");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }
  _simulation->getParams().lockMutex();
  _simulation->getParams().writeToIniFile(fileName.toStdString());
  _simulation->getParams().unlockMutex();
}

void SimControlPanel::on_loadSimulatorButton_clicked() {
  QString fileName = QFileDialog::getOpenFileName(nullptr, ("Save Simulator Table Values"), "../config", "All Files (*)");
  if(fileName == nullptr || fileName == "") {
    createErrorMessage("File name is invalid");
    return;
  }

  _simulation->getParams().lockMutex();
  _simulation->getParams().collection.clearAllSet();
  _simulation->getParams().initializeFromIniFile(fileName.toStdString());
  if(!_simulation->getParams().collection.checkIfAllSet()) {
    printf("new settings file %s doesn't contain the following simulator parameters:\n%s\n",
            fileName.toStdString().c_str(), _simulation->getParams().generateUnitializedList().c_str());
    throw std::runtime_error("bad new settings file");
  }
  loadSimulationParameters(_simulation->getParams());
  _simulation->getParams().unlockMutex();
}

void SimControlPanel::on_robotTable_cellChanged(int row, int column) {
  (void) row;
  (void) column;
}

void SimControlPanel::on_saveRobotButton_clicked() {

}

void SimControlPanel::on_loadRobotButton_clicked() {

}

void SimControlPanel::on_favoritesTable_cellChanged(int row, int column) {
  (void) row;
  (void) column;
}

void SimControlPanel::on_loadFavoriteButton_clicked() {

}
