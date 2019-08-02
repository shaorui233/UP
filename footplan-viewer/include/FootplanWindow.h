#ifndef CHEETAH_SOFTWARE_FOOTPLANWINDOW_H
#define CHEETAH_SOFTWARE_FOOTPLANWINDOW_H


#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include "GraphSearch.h"

class FootplanWindow : public QWidget {
  Q_OBJECT

public:
  FootplanWindow();
  void loadDefaultSettings();
  void setInitialTrajectory(std::vector<InputTrajectoryState>* trajectory) {
    _trajectory = trajectory;
  }

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  void drawGrid(QPainter* painter);
  void drawInitialTrajectory(QPainter *painter);

  void drawRotatedRectangle(float x, float y, float theta, float x_size, float y_size, QPainter *painter);
  float pt2px_x(float pt);
  float pt2px_y(float pt);

  float window_x, window_y;
  float x_min, x_max, y_min, y_max;
  float gridCellSize;
  QLabel* test_label;

  std::vector<InputTrajectoryState>* _trajectory = nullptr;
};


#endif //CHEETAH_SOFTWARE_FOOTPLANWINDOW_H
