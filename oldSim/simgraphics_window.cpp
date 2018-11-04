#include "simgraphics_window.h"

#include <iostream>


using std::cout;
float RandomFloat(float min, float max)
{
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}

// simple shader based on old homework
// the lighting is wrong, but it's good enough for robot visualization
// the lighting is still somewhat reasonable (zooming out doesn't cause the cheetah to become blinding white)
static const char *vertexShaderSource =
        "attribute highp vec3 posAttr;\n"        // world position in
        "uniform lowp vec3 colAttr;\n"           // "color" in (material color)
        "attribute highp vec3 normAttr;\n"       // normal in
        "varying lowp vec4 col;\n"               // "color" output to frag shader
        "uniform highp mat4 matrix;\n"           // transformation matrix
        "varying vec3 normal;\n"                 // normal output
        "varying vec3 light_dir;\n"              // light direciton output
        "varying vec3 eye_dir;\n"                // vector to camera
        "varying vec3 pos_world;\n"              // position in world
        "void main() {\n"
        "   col = vec4(0.5 + 0.0 * normAttr,1.0);//colAttr;\n"
        "   col = vec4(colAttr,0.3) +vec4( 0.03 * normAttr,0.0);\n"  // this passes the color to the fragment shader
        "   gl_Position = matrix * vec4(posAttr,1);\n"            // does the transformation
        "   normal = (matrix * vec4(normAttr,0)).xyz;\n"          // also transform the normal (this is wrong, but close enough)
        "   pos_world = posAttr;\n"        // this is wrong - shouldn't include the project matrix...
        "}\n";


// same as the one above, but allows a variable color for checkerboard
static const char *vertexShaderSource_slow =
        "attribute highp vec3 posAttr;\n"   // world position in
        "attribute lowp vec3 colAttr;\n"    // "color" in (material color)
        "attribute highp vec3 normAttr;\n"  // normal in
        "varying lowp vec4 col;\n"          // "color" output to frag shader
        "uniform highp mat4 matrix;\n" // transformation matrix
        "varying vec3 normal;\n"  // normal output
        "varying vec3 light_dir;\n" // light direciton output
        "varying vec3 eye_dir;\n" // vector to camera
        "varying vec3 pos_world;\n"
        "void main() {\n"
        "   col = vec4(0.5 + 0.4 * normAttr,1.0);//colAttr;\n"
        "   col = vec4(colAttr,0.3) +vec4( 0.1 * normAttr,0.0);\n" // adds a cool rainbow color
        "   gl_Position = matrix * vec4(posAttr,1);\n"
        "   normal = (matrix * vec4(normAttr,0)).xyz;\n"
        "   pos_world = posAttr;\n"
        "}\n";


static const char *fragmentShaderSource =
        "varying lowp vec4 col;\n"   // color from vertex shader
        "varying vec3 pos_world;\n"  // position from vertex shader
        "varying vec3 normal;\n"     // normal from vertex shader
        "void main() {\n"
        "   vec3 light_pos = vec3(0,0,4);\n"                      // location of the light
        "   float light_dist = length(light_pos - pos_world);\n"  // distance from the light (using very wrong pos_world)
        "   vec3 light_color = vec3(1,1,1);\n"                    // light is white
        "   vec3 mat_ambient = vec3(.1,.1,.1);\n"                 // ambient lighting is white
        "   vec3 mat_spec    = vec3(.3,.3,.3);\n"
        "   vec3 n = normalize(normal);"                          // normalized normal
        "   vec3 l = normalize(light_pos);"                       // direction of light
        "   float angle_thing = clamp( dot(-n,l), 0., 1.);\n"     // normal and light dot product (clamp to zero if negative)
        "   gl_FragColor = col + vec4(mat_ambient*light_color*40.*angle_thing/(light_dist*light_dist),0);\n" // crappy shading
        "}\n";

SimGraphicsWindow::SimGraphicsWindow(QWindow *parent, int use_cheetah_3)
    : QWindow(parent)
    , m_animating(false)
    , m_context(0)
    , m_device(0)
    , m_program(0)
    , m_frame(0)
    , mini_graphics(use_cheetah_3)
{
    cout<<"[SIM GRAPHICS] New graphics window. \n";
    // set up graphics
    //mini_graphics.load_models(1);
    setSurfaceType(QWindow::OpenGLSurface);
}




void SimGraphicsWindow::initialize()
{
    // create the two shader programs
    m_program = new QOpenGLShaderProgram(this);
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_program->link();
    m_posAttr = m_program->attributeLocation("posAttr");;
    m_colAttr = m_program->uniformLocation("colAttr"); // note that this is a uniform here
    m_normAttr = m_program->attributeLocation("normAttr");
    m_matrixUniform = m_program->uniformLocation("matrix");


    m_slow_program = new QOpenGLShaderProgram(this);
    m_slow_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource_slow);
    m_slow_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_slow_program->link();
    m_posAttr_slow = m_slow_program->attributeLocation("posAttr");
    m_colAttr_slow = m_slow_program->attributeLocation("colAttr"); // note that this an attribute here
    m_normAttr_slow = m_slow_program->attributeLocation("normAttr");
    m_matrixUniform_slow = m_slow_program->uniformLocation("matrix");

    //glClearColor(0.0f, 0.2f, 0.2f, 0.0f);               // disgusting green background color
    glClearColor(58.f/256.f,110.f/256.f,165.f/256.f,0.f); // windows 2000 light blue
}

void SimGraphicsWindow::render(QPainter *painter)
{
    // some magic from the QT website to make it work on hi-dpi screens
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);


    // by default, use the not-slow shader
    m_program->bind();
    // this kicks off the process of getting the most recent robot state
    mini_graphics.update();

    //glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // loop through all the bodies to render
    // the inside is a terrible hack currently
    // it only sends each unique part once, making it faster.
    for(int kk = 0; kk < 14; kk++)
    {
        int mi = kk;
        // draw the checkerboard first
        // this is so we can do have the "ghost" transparent cheetah with the checkerboard in the background
        if(mi == 0) mi = 8;
        else if(mi == 8) mi = 0;
        // transformation matrix to be applied by GPU
        QMatrix4x4 matrix;
        matrix.setToIdentity();
        // qt default
        matrix.perspective(60.0f, 4.0f/3.0f, 0.001f, 50.0f);

        // camera zoom
        matrix.translate(0.f, 0.f, -.45f*zoom);
        // camera orbit (in degrees!)
        matrix.rotate(ry,1,0,0);
        matrix.rotate(rx,0,0,1);


        // mi = 8 is the checkerboard floor, which uses the other shader to get the colors correct
        if(mi == 8)
        {
            glDisable(GL_BLEND);
            // switch to other shader program
            m_program->release();
            m_slow_program->bind();

            // combine camera and model matrix and give to opengl
            m_slow_program->setUniformValue(m_matrixUniform_slow, matrix * mini_graphics.get_model_xform(mi));

            // give opengl the vertices/normals/colors
            glVertexAttribPointer(m_posAttr_slow, 3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_vertices(mi));
            glVertexAttribPointer(m_colAttr_slow, 3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_colors(mi));
            glVertexAttribPointer(m_normAttr_slow,3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_normals(mi));

            // draw
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glEnableVertexAttribArray(2);

            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            glDrawArrays(GL_TRIANGLES, 0, mini_graphics.get_number_of_faces(mi)*3 );


            glDisableVertexAttribArray(2);
            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(0);

            // switch back to normal shaders
            m_slow_program->release();
            m_program->bind();
            //glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        // combine camera and model matrix and give to opengl
        m_program->setUniformValue(m_matrixUniform, matrix * mini_graphics.get_model_xform(mi));
        // use the first color for the entire model
        float* color_array = mini_graphics.get_colors(mi);
        m_program->setUniformValue(m_colAttr, QVector3D(color_array[3],color_array[4],color_array[5]));

        //
        if(mi == 0 || mi == 4 || mi == 10)
        {
            // give oepngl the vertices/normals/colors
            glVertexAttribPointer(m_posAttr, 3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_vertices(mi));
            glVertexAttribPointer(m_normAttr,3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_normals(mi));

            // draw
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            for(int sub_index = 0; sub_index < 4; sub_index++)
            {
                m_program->setUniformValue(m_matrixUniform, matrix * mini_graphics.get_model_xform((mi + sub_index)));
                float* color_array = mini_graphics.get_colors(mi);
                m_program->setUniformValue(m_colAttr, QVector3D(color_array[3],color_array[4],color_array[5]));
                glDrawArrays(GL_TRIANGLES, 0, mini_graphics.get_number_of_faces(mi)*3 );

            }

            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(0);
        }
        //body
        else if(mi == 9)
        {
            // give oepngl the vertices/normals/colors
            glVertexAttribPointer(m_posAttr, 3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_vertices(mi));
            glVertexAttribPointer(m_normAttr,3, GL_FLOAT, GL_FALSE, 0, mini_graphics.get_normals(mi));

            // draw
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            glDrawArrays(GL_TRIANGLES, 0, mini_graphics.get_number_of_faces(mi)*3 );

            glDisableVertexAttribArray(1);
            glDisableVertexAttribArray(0);
        }

    }


    m_program->release();

    ++m_frame;
}

// on click, start orbiting
void SimGraphicsWindow::mousePressEvent(QMouseEvent *event)
{
    //cout<<"burp\n";
    orbiting = true;
    orbiting_x_start = event->pos().x();
    orbiting_y_start = event->pos().y();
}

// after release, save value from orbiting
void SimGraphicsWindow::mouseReleaseEvent(QMouseEvent *event)
{
    orbiting = false;
    rx_base = rx_base + pixel_to_rad * (event->pos().x() - orbiting_x_start);
    ry_base = ry_base + pixel_to_rad * (event->pos().y() - orbiting_y_start);
}

// while orbiting, rotate
void SimGraphicsWindow::mouseMoveEvent(QMouseEvent *event)
{
    if(!orbiting) return;
    rx = rx_base + pixel_to_rad * (event->pos().x() - orbiting_x_start);
    ry = ry_base + pixel_to_rad * (event->pos().y() - orbiting_y_start);
}

// scroll
void SimGraphicsWindow::wheelEvent(QWheelEvent *e)
{
    if(e->angleDelta().y() > 0)
    {
        if(zoom > 1)
            zoom = 0.8*zoom;
    }
    else
    {
        if(zoom < 10)
            zoom = 1.2*zoom;
    }
    //cout<<"scroll: "<<e->angleDelta().y()<<"\n";
}

// pass along the robot state to the graphics stuff
void SimGraphicsWindow::set_robot_state(CheetahState *state)
{
    mini_graphics.set_robot_state(state);
    //printf("from update, robot height is %.3f\n",state->xfb[6]);
}

// qt default rendering stuff
void SimGraphicsWindow::render()
{
    if(m_frame%60 == 0)
    {

        qint64 now = QDateTime::currentMSecsSinceEpoch();
        fps = (60.f *1000.f / (now-last_frame_ms));
        //cout<<"FPS: "<<(60.f *1000.f / (now-last_frame_ms))<<"\n";
        last_frame_ms = now;
    }

    if (!m_device)
        m_device = new QOpenGLPaintDevice;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_device->setSize(size());

    QPainter painter(m_device);
    render(&painter);
}

void SimGraphicsWindow::renderLater()
{
    requestUpdate();
}

bool SimGraphicsWindow::event(QEvent *event)
{
    switch (event->type()) {
    case QEvent::UpdateRequest:
        renderNow();
        return true;
    default:
        return QWindow::event(event);
    }
}

void SimGraphicsWindow::exposeEvent(QExposeEvent *event)
{
    Q_UNUSED(event);

    if (isExposed())
        renderNow();
}

void SimGraphicsWindow::renderNow()
{
    if (!isExposed())
        return;

    bool needsInitialize = false;

    if (!m_context) {
        m_context = new QOpenGLContext(this);
        m_context->setFormat(requestedFormat());
        m_context->create();

        needsInitialize = true;
    }

    m_context->makeCurrent(this);

    if (needsInitialize) {
        initializeOpenGLFunctions();
        initialize();
    }

    //render();

    m_context->swapBuffers(this);

    if (m_animating)
        renderLater();
}


void SimGraphicsWindow::setAnimating(bool animating)
{
    m_animating = animating;

    if (animating)
        renderLater();
}
