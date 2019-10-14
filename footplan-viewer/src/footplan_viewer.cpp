#include <cstdio>
#include <QtWidgets/QApplication>
#include "FootplanWindow.h"
#include "GraphSearch.h"

int main(int argc, char** argv) {
  QApplication qtApp(argc, argv);
  FootstepPlanner planner(true);
  planner.buildInputTrajectory(2,0.03, {{0,0},{1,0},0.2}, 0.2);

  FootplanWindow window;
  window.setInitialTrajectory(&planner.getInitialTrajectory());
  window.show();
  qtApp.exec();
  return 0;
}