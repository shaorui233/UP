/*! @file Graphics3D.h
 *  @brief Visualizer window for simulator
 *
 *  This class displays a window for 3D graphics. It also implements scroll/pan/zoom.
 *  This uses OpenGL and QT.
 */

#ifndef PROJECT_GRAPHICS3D_H
#define PROJECT_GRAPHICS3D_H

#include "obj_loader.h"
#include "DrawList.h"
#include "FirstOrderIIRFilter.h"

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

#include <mutex>

class Graphics3D: public QWindow, protected QOpenGLFunctions {
Q_OBJECT
public:
  explicit Graphics3D(QWindow* parent = 0);
  virtual ~Graphics3D() { }
  //virtual void render(QPainter* painter);
  virtual void render();
  virtual void initialize();
  void setAnimating(bool animating);
  size_t setupCheetah3();
  size_t setupMiniCheetah();

  void lockGfxMutex() {
    _gfxMutex.lock();
  }

  void unlockGfxMutex() {
    _gfxMutex.unlock();
  }

  // set robot state
  double _fps = 0;
  DrawList _drawList;
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
  void keyReleaseEvent(QKeyEvent *e) override;
  void keyPressEvent(QKeyEvent *event) override;
//    void resizeEvent(QResizeEvent *event) override;



private:
  std::mutex _gfxMutex;
  void updateCameraMatrix();
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
  float _ry = -90;
  float _pixel_to_rad = .3f;
  float _zoom = 1;

  bool _rotOrig = true;

  QMatrix4x4 _cameraMatrix;
  Vec3<float> _v0;
  FirstOrderIIRFilter<Vec3<float>, float> _freeCamFilter;

  float _freeCamMove [3] = {0,0,0};
  float _freeCamPos [3] = {0.f,0.f,0.f};
  float _frameTime = 1.f/60.f;

  bool _arrowsPressed [4] = {false,false,false,false};

  float _targetSpeed = 2;

};


#endif //PROJECT_GRAPHICS3D_H
