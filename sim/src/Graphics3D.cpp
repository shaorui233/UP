/*! @file Graphics3D.cpp
 *  @brief Visualizer window for simulator
 *
 *  This class displays a window for 3D graphics. It also implements scroll/pan/zoom.
 */

#include "Graphics3D.h"
#include "utilities.h"

#include <iostream>
#include <unistd.h>



static constexpr auto clearColor = windows2000;

static constexpr char vertexShaderSource[] = R"(

// inputs:
attribute highp vec3 posAttr;   // position
attribute lowp vec3 colAttr;      // color
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
Graphics3D::Graphics3D(QWidget *parent) : QOpenGLWidget(parent), _animating(false),  _program(0),
                                          _frame(0), _v0(0, 0, 0), _freeCamFilter(1, 60, _v0) {
  std::cout << "[SIM GRAPHICS] New graphics window. \n";



  //setSurfaceType(QWindow::OpenGLSurface);
}

Graphics3D::~Graphics3D(){
}

/*!
 * Configure the window for displaying cheetah 3
 */
size_t Graphics3D::setupCheetah3() {
  return _drawList.addCheetah3();
}

/*!
 * Configure the window for displaying mini cheetah
 */
size_t Graphics3D::setupMiniCheetah() {
  return _drawList.addMiniCheetah();
}

/*!
 * Update the camera matrix for the current zoom/orbit
 */

void Graphics3D::updateCameraMatrix() {
  _cameraMatrix.setToIdentity();
  _cameraMatrix.perspective(60.f, float(size().width()) / float(size().height()), .001f, 50.f);

  if(_arrowsPressed[0]) _ry -= _targetSpeed/2.f;
  if(_arrowsPressed[1]) _ry += _targetSpeed/2.f;
  if(_arrowsPressed[2]) _rx += _targetSpeed/2.f;
  if(_arrowsPressed[3]) _rx -= _targetSpeed/2.f;
  if (!_rotOrig) {
    _ry = coerce<float>(_ry, -180, 0);
    // velocity in camera coordinates
    // we want the inverse transformation (coordinateRotation goes the opposite way as QMatrix.rotate())
    RotMat<float> R = coordinateRotation<float>(CoordinateAxis::Z, deg2rad(_rx)) *
                      coordinateRotation<float>(CoordinateAxis::X, deg2rad(_ry));
    Vec3<float> v(_freeCamMove[0], _freeCamMove[1], _freeCamMove[2]);
    v = R * v;

    // integrate and filter
    _freeCamFilter.update(v);
    for (size_t i = 0; i < 3; i++)
      _freeCamPos[i] += _frameTime * _freeCamFilter.get()[i];

    // apply
    _cameraMatrix.rotate(_ry, 1, 0, 0);
    _cameraMatrix.rotate(_rx, 0, 0, 1);
    _cameraMatrix.translate(_freeCamPos[0], _freeCamPos[1], _freeCamPos[2]);
  } else {
    _cameraMatrix.translate(0.f, 0.f, -.45f * _zoom);
    _cameraMatrix.rotate(_ry, 1, 0, 0);
    _cameraMatrix.rotate(_rx, 0, 0, 1);
  }
}

void Graphics3D::initializeGL() {
  std::cout << "[Graphics3D] Initialize OpenGL...\n";
  initializeOpenGLFunctions();
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
  _rx_base = _rx;
  _ry_base = _ry;
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
      _zoom = 0.8f * _zoom;
  } else {
    if (_zoom < 100)
      _zoom = 1.2f * _zoom;
  }
}

void Graphics3D::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Control) {
    _targetSpeed *= .5f;
  } else if (e->key() == Qt::Key_Shift) {
    _targetSpeed *= 2.f;
  }
  if (e->key() == Qt::Key_W) _freeCamMove[2] = _targetSpeed;
  else if (e->key() == Qt::Key_S) _freeCamMove[2] = -_targetSpeed;

  if (e->key() == Qt::Key_A) _freeCamMove[0] = _targetSpeed;
  else if (e->key() == Qt::Key_D) _freeCamMove[0] = -_targetSpeed;

  if (e->key() == Qt::Key_R) _freeCamMove[1] = -_targetSpeed;
  else if (e->key() == Qt::Key_F) _freeCamMove[1] = _targetSpeed;

  if (e->key() == Qt::Key_Up) _arrowsPressed[0] = true;
  else if (e->key() == Qt::Key_Down) _arrowsPressed[1] = true;

  if (e->key() == Qt::Key_Right) _arrowsPressed[2] = true;
  else if (e->key() == Qt::Key_Left) _arrowsPressed[3] = true;
  
  if (e->key() == Qt::Key_V) {
      if(_pause) _pause = false;
      else _pause = true;
  }


  if (e->key() == Qt::Key_Tab){
    _freeCamPos[0] = 0.f;
    _freeCamPos[1] = 0.f;
    _freeCamPos[2] = 0.f;
    _rx = 0.f;
    _ry = 0.f;
  }
}

void Graphics3D::keyReleaseEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Control) {
    _targetSpeed /= .5f;
  } else if (e->key() == Qt::Key_Shift) {
    _targetSpeed /= 2.f;
  } else if (e->key() == Qt::Key_Space) {
    _rotOrig = !_rotOrig;
  }

  if (e->key() == Qt::Key_W) _freeCamMove[2] = 0;
  else if (e->key() == Qt::Key_S) _freeCamMove[2] = 0;

  if (e->key() == Qt::Key_A) _freeCamMove[0] = 0;
  else if (e->key() == Qt::Key_D) _freeCamMove[0] = 0;

  if (e->key() == Qt::Key_R) _freeCamMove[1] = 0;
  else if (e->key() == Qt::Key_F) _freeCamMove[1] = 0;

  if (e->key() == Qt::Key_Up) _arrowsPressed[0] = false;
  else if (e->key() == Qt::Key_Down) _arrowsPressed[1] = false;

  if (e->key() == Qt::Key_Right) _arrowsPressed[2] = false;
  else if (e->key() == Qt::Key_Left) _arrowsPressed[3] = false;
}

/*!
 * Enable and disable animation
 */
void Graphics3D::setAnimating(bool animating) {
  _animating = animating;
}



GLuint buffID[3];
void Graphics3D::paintGL() {
  // update joystick:
  _gameController.updateDriverCommand(_driverCommand);
  if(!_animating) return;
  if (_frame % 60 == 0) {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    _fps = (60.f * 1000.f / (now - last_frame_ms));
    std::cout << "FPS: " << _fps << "\n";
    last_frame_ms = now;
  }
  QPainter painter2(this);


  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


  // magic copied from the internet to make things look good on hi-dpi screens
  //const qreal retinaScale = devicePixelRatio();
  //glViewport(0, 0, width() * retinaScale, height() * retinaScale);

  updateCameraMatrix();
  _program->bind();


  if (_drawList.needsReload()) {
    // upload data

    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    glGenBuffers(3, buffID);

    glBindBuffer(GL_ARRAY_BUFFER, buffID[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _drawList.getSizeOfAllData(), _drawList.getVertexArray(),
                 GL_STATIC_DRAW);
    printf("size: %lu, @ %p\n", _drawList.getSizeOfAllData(), _drawList.getVertexArray());
    glVertexAttribPointer(_posAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, buffID[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _drawList.getSizeOfAllData(), _drawList.getColorArray(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(_colAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, buffID[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * _drawList.getSizeOfAllData(), _drawList.getNormalArray(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(_normAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    printf("[Graphics 3D] Uploaded data (%f MB)\n", _drawList.getGLDataSizeMB());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glBindBuffer(GL_ARRAY_BUFFER, buffID[0]);
  glVertexAttribPointer(_posAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glBindBuffer(GL_ARRAY_BUFFER, buffID[1]);
  glVertexAttribPointer(_colAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glBindBuffer(GL_ARRAY_BUFFER, buffID[2]);
  glVertexAttribPointer(_normAttr, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);
  for (size_t id = 0; id < _drawList.getNumObjectsToDraw(); id++) {
    _program->setUniformValue(_matrixUniform, 
            _cameraMatrix * _drawList.getModelKinematicTransform(id) *
            _drawList.getModelBaseTransform(id));
    glDrawArrays(GL_TRIANGLES, _drawList.getGLDrawArrayOffset(id) / 3, 
            _drawList.getGLDrawArraySize(id) / 3);
  }
  glDisableVertexAttribArray(2);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  _program->release();
  /************     OpenGL Drawing (not in drawlist object data)     **********/
  _BoxObstacleDrawing();
  // Useful information drawing
  _Additional_Drawing();
  /********************       End of OpenGL Drawing        ********************/
  ++_frame;
  glDisable(GL_DEPTH_TEST);
  painter2.setPen(QColor(100,100,100,255));
  painter2.fillRect(QRect(30,30,400,200), QColor(100,100,100,220));
  QFont font("Monospace", 20);
  painter2.setPen(QColor(210, 100, 100));
  painter2.setFont(font);
  painter2.drawText(QRect(30,30,1000,1000), Qt::AlignLeft, QString(infoString));

  painter2.end();
}
void Graphics3D::_BoxObstacleDrawing(){
    glLoadMatrixf(_cameraMatrix.data());
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    size_t nBox = _drawList.getBoxInfoList().size();
    for(size_t i(0); i < nBox; ++i){
        glPushMatrix();
        glMultMatrixf(_drawList.getBoxInfoList()[i].frame);
        _DrawBox(_drawList.getBoxInfoList()[i].depth, 
                _drawList.getBoxInfoList()[i].width, 
                _drawList.getBoxInfoList()[i].height);
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
    glPopAttrib();
}

void Graphics3D::_DrawBox(double depth, double width, double height)
{
    double x = depth/2.0;
    double y = width/2.0;
    double z = height/2.0;

    glPushAttrib(GL_COLOR_BUFFER_BIT);

    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

    glColor4f(_color1[0], _color1[1], _color1[2], 0.7f);

    glBegin(GL_QUADS);
	{
		glVertex3d( x,  y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x,  y,  z);
		glVertex3d( x,  y,  z);

		glVertex3d( x, -y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x,  y,  z);
		glVertex3d( x, -y,  z);

		glVertex3d( x, -y,  z);
		glVertex3d(-x, -y,  z);
		glVertex3d(-x, -y, -z);
		glVertex3d( x, -y, -z);

		glVertex3d(-x, -y,  z);
		glVertex3d(-x,  y,  z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x, -y, -z);

		glVertex3d(-x, -y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x, -y, -z);

		glVertex3d(-x, -y,  z);
		glVertex3d( x, -y,  z);
		glVertex3d( x,  y,  z);
		glVertex3d(-x,  y,  z);
	} //GL_QUADS
	glEnd();
    glPopAttrib();
    glDisable(GL_BLEND);
}


void Graphics3D::_Additional_Drawing(){
    glLoadMatrixf(_cameraMatrix.data());
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    _DrawContactForce();
    _DrawContactPoint();

    glPopAttrib();
    glEnable(GL_LIGHTING);
}


void Graphics3D::_DrawContactForce(){
    glLineWidth(2.0);
    //double scale(0.02);
    double scale(20.);

	glPushAttrib(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

    for(size_t i(0); i<_drawList.getTotalNumGC(); ++i){
        glBegin(GL_LINES);
        glColor4f(0.8f, 0.0f, 0.f, 0.5f);

        glVertex3f(_drawList.getGCPos(i)[0], 
                _drawList.getGCPos(i)[1], 
                _drawList.getGCPos(i)[2]);

        glVertex3f(_drawList.getGCPos(i)[0] + scale * _drawList.getGCForce(i)[0],
                _drawList.getGCPos(i)[1] + scale * _drawList.getGCForce(i)[1],
                _drawList.getGCPos(i)[2] + scale * _drawList.getGCForce(i)[2]);

        glEnd();
    }
    glPopAttrib();
    glDisable(GL_BLEND);
}

void Graphics3D::_DrawContactPoint(){
    glPointSize(5);

	glPushAttrib(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

    for(size_t i(0); i<_drawList.getTotalNumGC(); ++i){
        glBegin(GL_POINTS);
        glColor4f(0.8f, 0.0f, 0.1f, 0.3f);

        glVertex3f(_drawList.getGCPos(i)[0], 
                _drawList.getGCPos(i)[1], 
                _drawList.getGCPos(i)[2]);

        glEnd();
    }
    glPopAttrib();
    glDisable(GL_BLEND);
}
