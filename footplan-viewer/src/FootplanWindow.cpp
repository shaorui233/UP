#include <QtWidgets/QGridLayout>
#include <QtGui/QPainter>
#include <cmath>
#include <Math/orientation_tools.h>
#include "FootplanWindow.h"

FootplanWindow::FootplanWindow() {
  // for now, hardcoded settings
  loadDefaultSettings();

  test_label = new QLabel("test label!");
  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->setColumnStretch(0, 1);
  mainLayout->setColumnStretch(3, 1);
  mainLayout->addWidget(test_label);
  setLayout(mainLayout);
  setWindowTitle("Foot Plan Viewer");
  setFixedSize(window_x, window_y);
}

void FootplanWindow::loadDefaultSettings() {
  // window size
  window_x = 1024;
  window_y = 720;

  // display size
  x_min = -2;
  x_max = 2;
  y_min = -2 * (window_y / window_x);
  y_max = -y_min;

  gridCellSize = 0.5;
}

void FootplanWindow::paintEvent(QPaintEvent *event) {
  printf("Paint!\n");
  QPainter painter(this);

  //painter.drawRect(10,10,100,100);
  drawGrid(&painter);
  drawInitialTrajectory(&painter);

  (void)event;
}

float FootplanWindow::pt2px_x(float pt) {
  return (pt - x_min) * window_x / (x_max - x_min);
}

float FootplanWindow::pt2px_y(float pt) {
  return (pt - y_min) * window_y / (y_max - y_min);
}


void FootplanWindow::drawGrid(QPainter *painter) {
  // first round up to nearest even increment
  float x_draw = gridCellSize * std::ceil(x_min / gridCellSize);
  float y_draw = gridCellSize * std::ceil(y_min / gridCellSize);

  x_draw = pt2px_x(x_draw);
  y_draw = pt2px_y(y_draw);

  float x_step = gridCellSize * window_x / (x_max - x_min);
  float y_step = gridCellSize * window_y / (y_max - y_min);

  while(x_draw < window_x) {
    painter->drawLine(x_draw,0,x_draw,window_y);
    x_draw += x_step;
  }

  while(y_draw < window_y) {
    painter->drawLine(0, y_draw, window_x, y_draw);
    y_draw += y_step;
  }

  painter->drawEllipse(pt2px_x(0) - 10, pt2px_y(0) - 10, 20, 20);
}

void FootplanWindow::drawInitialTrajectory(QPainter *painter) {
  if(!_trajectory || _trajectory->size() < 2) return;


  for(uint32_t i = 1; i < _trajectory->size(); i++) {
    auto& n = (*_trajectory)[i];
    auto& p = (*_trajectory)[i - 1];

    painter->drawLine(pt2px_x(p.p[0]), pt2px_y(p.p[1]), pt2px_x(n.p[0]), pt2px_y(n.p[1]));
    painter->drawRect(pt2px_x(n.p[0]) - 1, pt2px_y(n.p[1]) - 1, 2, 2);

    if(!(i % 16)) {
      drawRotatedRectangle(p.p[0], p.p[1], p.theta, 0.19 * 2, 0.1 * 2, painter);
    }
  }
}

void FootplanWindow::drawRotatedRectangle(float x, float y, float theta, float x_size, float y_size, QPainter *painter) {
  Vec3<float> corners[4] = {{- x_size/2, - y_size/2, 0},
                            {- x_size/2, + y_size/2, 0},
                            {+ x_size/2, + y_size/2, 0},
                            {+ x_size/2, - y_size/2, 0}};

  Vec3<float> origin = {x,y,0};

  Mat3<float> R = ori::coordinateRotation(ori::CoordinateAxis::Z, theta);
  for(auto& corner : corners) {
    corner = R.transpose() * corner + origin;
  }

  for(uint32_t i = 0; i < 4; i++) {
    auto& p = corners[i];
    auto& n = corners[(i + 1) % 4];
    painter->drawLine(pt2px_x(p[0]), pt2px_y(p[1]), pt2px_x(n[0]), pt2px_y(n[1]));
  }


}