/*! @file Graphics3D.cpp
 *  @brief Visualizer window for simulator
 *
 *  This class displays a window for 3D graphics. It also implements scroll/pan/zoom.
 */

#include "Graphics3D.h"
#include <iostream>
#include <include/Graphics3D.h>
#include <unistd.h>



static constexpr auto clearColor = windows2000;

static constexpr char vertexShaderSource[] = R"(

// inputs:
attribute highp vec3 posAttr;   // position
uniform lowp vec3 colAttr;      // color
attribute highp vec3 normAttr;  // normal
uniform highp mat4 matrix;      // transformation

// outputs:
varying lowp vec4 col;          // color
varying vec3 normal;            // normal
varying vec3 pos_world;         // position

void main() {
  col = vec4(colAttr,0.3);
  gl_Position = matrix * vec4(posAttr,1);
  normal = (matrix * vec4(normAttr,0)).xyz;
  pos_world = posAttr;
}
)";

static constexpr char fragmentShaderSource[] = R"(
varying lowp vec4 col;
varying vec3 pos_world;
varying vec3 normal;
void main() {
  vec3 light_pos = vec3(0,0,4);
  float light_dist = length(light_pos - pos_world);
  vec3 light_color = vec3(1,1,1);
  vec3 mat_ambient = vec3(.1,.1,.1);
  vec3 mat_spec    = vec3(.3,.3,.3);
  vec3 n = normalize(normal);
  vec3 l = normalize(light_pos);
  float angle_thing = clamp( dot(-n,l), 0., 1.);
  gl_FragColor = col + vec4(mat_ambient*light_color*40.*angle_thing/(light_dist*light_dist),0);
}
)";

/*!
 * Initialize a 3D visualization window
 */
Graphics3D::Graphics3D(QWindow *parent) : QWindow(parent), _animating(false), _context(0), _device(0), _program(0),
                                          _frame(0) {
  std::cout << "[SIM GRAPHICS] New graphics window. \n";
  setSurfaceType(QWindow::OpenGLSurface);
}

/*!
 * Configure the window for displaying cheetah 3
 */
size_t Graphics3D::setupCheetah3() {
  return _drawList.loadCheetah3();
}

/*!
 * Configure the window for displaying mini cheetah
 */
size_t Graphics3D::setupMiniCheetah() {
  return _drawList.loadMiniCheetah();
}

/*!
 * Update the camera matrix for the current zoom/orbit
 */
void Graphics3D::updateCameraMatrix() {
  _cameraMatrix.setToIdentity();
  // todo set aspect ratio
  _cameraMatrix.perspective(60.f, float(size().width()) / float(size().height()), .001f, 50.f);
  _cameraMatrix.translate(0.f, 0.f, -.45f * _zoom);
  _cameraMatrix.rotate(_ry, 1, 0, 0);
  _cameraMatrix.rotate(_rx, 0, 0, 1);
}

/*!
 * Draw a frame with OpenGL
 */
void Graphics3D::render(QPainter *painter) {
  (void) painter;

  updateCameraMatrix();
  _program->bind();

  if (_drawList.needsReload()) {
    // upload data
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);
    glVertexAttribPointer(_posAttr, 3, GL_FLOAT, GL_FALSE, 0, _drawList.getVertexArray());
    glVertexAttribPointer(_colAttr, 3, GL_FLOAT, GL_FALSE, 0, _drawList.getColorArray());
    glVertexAttribPointer(_normAttr, 3, GL_FLOAT, GL_FALSE, 0, _drawList.getNormalArray());
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    printf("[Graphics 3D] Uploaded data (%f MB)\n", _drawList.getGLDataSizeMB());
  }

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  for (size_t id = 0; id < _drawList.getNumObjectsToDraw(); id++) {
    _program->setUniformValue(_matrixUniform, _cameraMatrix * _drawList.getModelKinematicTransform(id) * _drawList.getModelBaseTransform(id));
    glDrawArrays(GL_TRIANGLES, _drawList.getGLDrawArrayOffset(id) / 3, _drawList.getGLDrawArraySize(id) / 3);
  }

  _program->release();
  ++_frame;
}


/*!
 * Initialize OpenGL and load shaders
 */
void Graphics3D::initialize() {
  std::cout << "[Graphics3D] Initialize OpenGL...\n";

  // create GPU shaders
  _program = new QOpenGLShaderProgram(this);
  _program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
  _program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
  _program->link();

  // setup attributes/uniforms (inputs to shaders)
  _posAttr = (GLuint) _program->attributeLocation("posAttr");
  _colAttr = (GLuint) _program->attributeLocation("colAttr");
  _normAttr = (GLuint) _program->attributeLocation("normAttr");
  _matrixUniform = (GLuint) _program->uniformLocation("matrix");

  // set clear color:
  glClearColor(clearColor[0], clearColor[1], clearColor[2], 0.f);
}

/*-----------------------------------------*
 * Mouse Handlers for Orbit and Zoom       *
 *-----------------------------------------*/

void Graphics3D::mousePressEvent(QMouseEvent *event) {
  _orbiting = true;
  _orbiting_x_start = event->pos().x();
  _orbiting_y_start = event->pos().y();
}

void Graphics3D::mouseMoveEvent(QMouseEvent *event) {
  if (!_orbiting) return;
  _rx = _rx_base + _pixel_to_rad * (event->pos().x() - _orbiting_x_start);
  _ry = _ry_base + _pixel_to_rad * (event->pos().y() - _orbiting_y_start);
}

void Graphics3D::mouseReleaseEvent(QMouseEvent *event) {
  _orbiting = false;
  _rx_base = _rx_base + _pixel_to_rad * (event->pos().x() - _orbiting_x_start);
  _ry_base = _ry_base + _pixel_to_rad * (event->pos().y() - _orbiting_y_start);
}

void Graphics3D::wheelEvent(QWheelEvent *e) {
  if (e->angleDelta().y() > 0) {
    if (_zoom > .1)
      _zoom = 0.8 * _zoom;
  } else {
    if (_zoom < 10)
      _zoom = 1.2 * _zoom;
  }
}

/*-----------------------------------------*
 * Confusing QT Stuff to set up the window *
 *-----------------------------------------*/

/*!
 * Wrapper for our rendering function which takes care of some OpenGL and Qt setup
 */
void Graphics3D::render() {
  if (_frame % 60 == 0) {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    _fps = (60.f * 1000.f / (now - last_frame_ms));
    std::cout << "FPS: " << _fps << "\n";
    last_frame_ms = now;
  }

  if (!_device)
    _device = new QOpenGLPaintDevice;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  _device->setSize(size());

  QPainter painter(_device);

  // magic copied from the internet to make things look good on hi-dpi screens
  const qreal retinaScale = devicePixelRatio();
  glViewport(0, 0, width() * retinaScale, height() * retinaScale);

  render(&painter);
}

/*!
 * Enable and disable animation
 */
void Graphics3D::setAnimating(bool animating) {
  _animating = animating;
  if (animating)
    renderLater();
}

/*!
 * Called by Qt event handlers, does initialization if needed and draws a frame.
 */
void Graphics3D::renderNow() {

  if (!isExposed()) // window isn't ready
    return;
  _gfxMutex.lock();
  bool needsInitialize = false;

  // initialize things if we need to
  if (!_context) {
    std::cout << "[Graphics3D] Initialize context...\n";
    _context = new QOpenGLContext(this);
    _context->setFormat(requestedFormat());
    _context->create();

    needsInitialize = true;
  }

  _context->makeCurrent(this);

  if (needsInitialize) {
    initializeOpenGLFunctions();
    initialize();
  }

  // draw frame
  render();
  _context->swapBuffers(this);

  // if we're running, we should schedule the next frame
  if (_animating)
    renderLater();
  _gfxMutex.unlock();
}

/*!
 * Sets up Qt to call event with an UpdateRequest on the next frame, which will run the frame drawing code
 */
void Graphics3D::renderLater() {
  requestUpdate();
}

/*!
 * This event handler is called automatically by the QT framework at the appropriate time
 * and runs the frame drawing code.
 */
bool Graphics3D::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
      renderNow();
      return true;
    default:
      return QWindow::event(event);
  }
}

/*!
 * This function is called once the window has initialized all the way and renders the first frame
 */
void Graphics3D::exposeEvent(QExposeEvent *event) {
  (void) event;
  if (isExposed())
    renderNow();
}


