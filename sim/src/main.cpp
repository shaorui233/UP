/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include <stdio.h>
#include <QApplication>


#include <Graphics3D.h>
#include <QSurfaceFormat>

QSurfaceFormat gFormat;

/*!
 * Hack to launch a simulator graphics window
 */
 int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  printf("hello world!\n");
  gFormat.setSamples(1);
  gFormat.setDepthBufferSize(24);
  Graphics3D* window = new Graphics3D();
  window->setFormat(gFormat);
  window->show();
  window->setAnimating(true);
  window->resize(1280,720);
  return a.exec();
 }
