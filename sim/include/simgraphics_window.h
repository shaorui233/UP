#ifndef SIMGRAPHICSWINDOW_H
#define SIMGRAPHICSWINDOW_H

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


#include "obj_loader.h"
#include "mini_cheetah_graphics.h"
#include "simulator_interface.h"


// qt window for simulation graphics
// based on qt tutorial for the setAnimating and renderNow framework
// automatically draws frames at 60 Hz
class SimGraphicsWindow: public QWindow, protected QOpenGLFunctions
{
    //Q_OBJECT for some reason this breaks things
public:
    explicit SimGraphicsWindow(QWindow* parent = 0, int use_cheetah_3=1);
    //~SimGraphicsWindow();
    virtual ~SimGraphicsWindow() {}

    virtual void render(QPainter *painter);
    virtual void render();

    virtual void initialize();

    void setAnimating(bool animating);
    void set_robot_state(CheetahState* state);
    float fps = 0.f;

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
    bool m_animating;

    QOpenGLContext *m_context;
    QOpenGLPaintDevice *m_device;

    // attributes for shader program
    GLuint m_posAttr; // position of vertex
    GLuint m_colAttr; // color of vertex
    GLuint m_matrixUniform; // transformation matrix
    GLuint m_normAttr; // vertex normal

    // there is a slower shader used only to draw the checkerboard
    // I suspect the first shader is only really slow on my computer
    // which has a weird graphics issue where transferring data to the
    // GPU is stupidly slow
    GLuint m_posAttr_slow;
    GLuint m_colAttr_slow;
    GLuint m_matrixUniform_slow;
    GLuint m_normAttr_slow;

    // shader programs
    // m_program can only draw items with a uniform material color
    // m_slow_program can do per-vertex color
    QOpenGLShaderProgram *m_program;
    QOpenGLShaderProgram *m_slow_program;

    // frame count
    int m_frame;
    // time of last frame
    qint64 last_frame_ms = 0;

    /* JUNK */
    GLfloat* vertices_test;
    GLfloat* colors_test;
    int num_tris = 3000;
    GLfloat base_loc[2];
    GLuint VertexArrayID;
    ObjLoader objLoader;

    // UI orbit/zoom variables
    bool orbiting = false;
    int orbiting_x_start = 0;
    int orbiting_y_start = 0;
    float rx_base = 0;
    float ry_base = 0;
    float rx = 0;
    float ry = 0;
    float pixel_to_rad = .3f;
    float zoom = 1;

    // graphics object to load cheetah graphics
    mini_cheetah_graphics mini_graphics;

};

#endif // SIMGRAPHICSWINDOW_H
