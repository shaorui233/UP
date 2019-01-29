#include "SimControlPanel.h"
#include "ui_SimControlPanel.h"

SimControlPanel::SimControlPanel(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimControlPanel) {
  ui->setupUi(this);
}

SimControlPanel::~SimControlPanel() {
  delete ui;
}

void SimControlPanel::on_startButton_clicked() {

}

void SimControlPanel::on_stopButton_clicked() {

}

void SimControlPanel::on_joystickButton_clicked() {

}

void SimControlPanel::on_driverButton_clicked() {

}

void SimControlPanel::on_simulatorTable_cellChanged(int row, int column) {
  (void) row;
  (void) column;
}

void SimControlPanel::on_saveSimulatorButton_clicked() {

}

void SimControlPanel::on_loadSimulatorButton_clicked() {

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
