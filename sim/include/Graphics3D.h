/*! @file Graphics3D.h
 *  @brief Visualizer window for simulator
 *
 *  This class displays a window for 3D graphics. It also implements scroll/pan/zoom.
 *  This uses OpenGL and QT.
 */

#ifndef PROJECT_GRAPHICS3D_H
#define PROJECT_GRAPHICS3D_H

#include <QWindow>
#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>
#include <QOpenGLContext>
#include <QPainter>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>

#include <QWheelEvent>
#include <QScreen>
#include <QDateTime>
#include <QMouseEvent>

#include <obj_loader.h>

class Graphics3D: public QWindow, protected QOpenGLFunctions {
Q_OBJECT
public:
  explicit Graphics3D(QWindow* parent = 0);
  virtual ~Graphics3D() { }
  virtual void render(QPainter* painter);
  virtual void render();
  virtual void initialize();
  void setAnimating(bool animating);
  // set robot state
  double _fps = 0;
public slots:
  void renderLater();
  void renderNow();
protected:
  bool event(QEvent *event) override;

  void exposeEvent(QExposeEvent *event) override;

  // mouse callbacks for orbit and zoom
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *e) override;
//    void resizeEvent(QResizeEvent *event) override;

private:
  bool _animating;

  QOpenGLContext *_context;
  QOpenGLPaintDevice *_device;

  // attributes for shader program
  GLuint _posAttr; // position of vertex
  GLuint _colAttr; // color of vertex
  GLuint _matrixUniform; // transformation matrix
  GLuint _normAttr; // vertex normal
  GLuint _posAttr_slow;
  GLuint _colAttr_slow;
  GLuint _matrixUniform_slow;
  GLuint _normAttr_slow;

  // shader programs
  QOpenGLShaderProgram *_program;
  QOpenGLShaderProgram *_slow_program;

  // frame count
  int _frame;
  // time of last frame
  qint64 last_frame_ms = 0;

  // UI orbit/zoom variables
  bool _orbiting = false;
  int _orbiting_x_start = 0;
  int _orbiting_y_start = 0;
  float _rx_base = 0;
  float _ry_base = 0;
  float _rx = 0;
  float _ry = 0;
  float _pixel_to_rad = .3f;
  double _zoom = 1;
};


#endif //PROJECT_GRAPHICS3D_H
