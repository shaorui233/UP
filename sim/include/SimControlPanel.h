#ifndef SIMCONTROLPANEL_H
#define SIMCONTROLPANEL_H

#include <QMainWindow>

namespace Ui {
class SimControlPanel;
}

class SimControlPanel : public QMainWindow
{
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

private:
    Ui::SimControlPanel *ui;
};

#endif // SIMCONTROLPANEL_H
