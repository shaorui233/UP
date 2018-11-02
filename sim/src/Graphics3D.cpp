/*! @file Graphics3D.cpp
 *  @brief Visualizer window for simulator
 *
 *  This class displays a window for 3D graphics. It also implements scroll/pan/zoom.
 */

#include "Graphics3D.h"
#include <iostream>
#include <include/Graphics3D.h>
#include <unistd.h>


Graphics3D::Graphics3D(QWindow *parent) : QWindow(parent), _animating(false), _context(0), _device(0), _program(0),
                                          _frame(0) {
  std::cout << "[SIM GRAPHICS] New graphics window. \n";
  setSurfaceType(QWindow::OpenGLSurface);
}

void Graphics3D::renderLater() {
  requestUpdate();
}

void Graphics3D::render(QPainter *painter) {
  (void)painter;
  ++_frame;
}



void Graphics3D::initialize() {
  std::cout << "Initialize called!\n";
}


void Graphics3D::mousePressEvent(QMouseEvent *event) {
  _orbiting = true;
  _orbiting_x_start = event->pos().x();
  _orbiting_y_start = event->pos().y();
}

void Graphics3D::mouseMoveEvent(QMouseEvent *event) {
  if(!_orbiting) return;
  _rx = _rx_base + _pixel_to_rad * (event->pos().x() - _orbiting_x_start);
  _ry = _ry_base + _pixel_to_rad * (event->pos().y() - _orbiting_y_start);
}

void Graphics3D::mouseReleaseEvent(QMouseEvent *event) {
  _orbiting = false;
  _rx_base = _rx_base + _pixel_to_rad * (event->pos().x() - _orbiting_x_start);
  _ry_base = _ry_base + _pixel_to_rad * (event->pos().y() - _orbiting_y_start);
}

void Graphics3D::wheelEvent(QWheelEvent *e) {
  if(e->angleDelta().y() > 0) {
    if(_zoom > 1)
      _zoom = 0.8*_zoom;
  } else {
    if(_zoom < 10)
      _zoom = 1.2*_zoom;
  }
}

/*-----------------------------------------*
 * Confusing QT Stuff to set up the window *
 *-----------------------------------------*/

void Graphics3D::render() {
  if(_frame%60 == 0) {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    _fps = (60.f *1000.f / (now-last_frame_ms));
    std::cout<<"FPS: "<<_fps<<"\n";
    last_frame_ms = now;
  }

  if (!_device)
    _device = new QOpenGLPaintDevice;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  _device->setSize(size());

  QPainter painter(_device);
  render(&painter);
}

/*!
 * Enable and disable animation
 */
void Graphics3D::setAnimating(bool animating) {
  _animating = animating;
  if(animating)
    renderLater();
}

void Graphics3D::renderNow() {
  if (!isExposed())
    return;

  bool needsInitialize = false;

  if (!_context) {
    std::cout << "Init context\n";
    _context = new QOpenGLContext(this);
    _context->setFormat(requestedFormat());
    _context->create();

    needsInitialize = true;
  }

  _context->makeCurrent(this);

  if (needsInitialize) {
    std::cout << "renderNow initializing...\n";
    initializeOpenGLFunctions();
    initialize();
  }

  render();

  _context->swapBuffers(this);

  if (_animating)
    renderLater();
}

bool Graphics3D::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
      renderNow();
      return true;
    default:
      return QWindow::event(event);
  }
}

void Graphics3D::exposeEvent(QExposeEvent *event) {
  (void)event;
  if (isExposed())
    renderNow();
}


