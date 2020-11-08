#include "qglViewerTest.h"

#if QT_VERSION >= 0x040000
#include <QKeyEvent>
#endif

using namespace qglviewer;
using namespace std;

// See:
//  - https://slideplayer.com/slide/15311230/

// Constructor must call the base class constructor.
qglViewerTest::qglViewerTest(QWidget *parent) : QGLViewer(parent)
{
    // Restore previous viewer state.
    //restoreStateFromFile();

    // Opens help window
    //help();

    /// Disable QGLViewer's default "spin"
    camera()->frame()->setSpinningSensitivity(100);
    camera()->frame()->setRotationSensitivity(1.0);
    camera()->frame()->setTranslationSensitivity(1.0);

    camera()->setZNearCoefficient(0.005);
    camera()->setZClippingCoefficient(100);

    /// Bindings @see QGLViewer::setDefaultMouseBindings()
    /// Extra behavior in this->mouseDoubleClickEvent()
    {
        /// Disable double click to align scene
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION, true);   /// ALIGN_CAMERA
        setMouseBinding(Qt::ShiftModifier, Qt::RightButton, NO_CLICK_ACTION);     /// RAP_FROM_PIXEL
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, NO_CLICK_ACTION, true); /// ZOOM_TO_FIT
    }

    /// Disable options that give OpenGL4 troubles
    {
        bool DISABLED = false;
        setShortcut(CAMERA_MODE, DISABLED);
        setShortcut(DRAW_AXIS, DISABLED);
        setShortcut(DRAW_GRID, DISABLED);
        setShortcut(DISPLAY_FPS, DISABLED);
        setShortcut(STEREO, DISABLED);
        setShortcut(ENABLE_TEXT, DISABLED);
        setShortcut(EDIT_CAMERA, DISABLED);
        setShortcut(ANIMATION, DISABLED);
        setShortcut(INCREASE_FLYSPEED, DISABLED);
        setShortcut(DECREASE_FLYSPEED, DISABLED);
        setShortcut(MOVE_CAMERA_LEFT, DISABLED);
        setShortcut(MOVE_CAMERA_RIGHT, DISABLED);
        setShortcut(MOVE_CAMERA_UP, DISABLED);
        setShortcut(MOVE_CAMERA_DOWN, DISABLED);
    }

    setAxisIsDrawn(false);
    setMouseTracking(true);

    // Texture parameters
    u_max = 1.0;
    v_max = 1.0;
    ratio = 1.0;
}

// Draws a spiral
void qglViewerTest::initializeGL()
{
    //QGLViewer::initializeGL();     //initializeOpenGLFunctions();

    // See https://www.trentreed.net/blog/qt5-opengl-part-1-basic-rendering/

    // See https://learnopengl.com/Getting-started/Shaders
    // Shaders for making circular points and varying size depending on distance to camera
    QOpenGLShader * vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
            "#version 410 compatibility\n"
            "uniform float pointSize;\n"
            "out vec4 colorV;\n"
            " \n"
            "void main() \n"
            "{\n"
            "    vec4 color = gl_Color;\n"
            "    vec4 position = gl_Vertex;\n"
            "    colorV = color;\n"
          //"    colorV = vec4(max(position.z/3,0) + 0.1, max(position.z/3,0) + 0.1, max(position.z/3,0) + 0.1, 1.0);\n"
            "    gl_Position = gl_ModelViewProjectionMatrix * position;\n"
            "    float scale = exp(-0.1*(gl_Position.z));\n"
            "    if (scale > 1)\n"
            "       scale = 1.0;\n"
            "    gl_PointSize = scale * pointSize;\n"
            "}";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader * fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
            "#version 410 compatibility\n"
        "#ifdef GL_OES_standard_derivatives\n"
        "#extension GL_OES_standard_derivatives : enable\n"
        "#endif\n"
            "in vec4 colorV;\n"
            "  \n"
            "void main()\n"
            "{\n"
            "    float fragZ = gl_FragCoord.z;\n"
            "    float scale = exp(-20*(1-fragZ));\n"
          //"    gl_FragColor = colorV;\n"
          //"    gl_FragColor = vec4(scale*colorV.x, scale*colorV.y, scale*colorV.z, 1);\n"
          //"    gl_FragColor = vec4(scale, scale, scale, 1);\n"
            "float r = 0.0, delta = 0.0, alpha = 1.0;\n"
            "vec2 cxy = 2.0 * gl_PointCoord - 1.0;\n" // see https://www.desultoryquest.com/blog/drawing-anti-aliased-circular-points-using-opengl-slash-webgl/
            "r = dot(cxy, cxy);\n"                    // using the gl_PointCoord requires "glEnable(GL_POINT_SPRITE);"
        "#ifdef GL_OES_standard_derivatives\n"
            "delta = fwidth(r);\n"
            "alpha = 1.0 - smoothstep(1.0 - delta, 1.0 + delta, r);\n"
        "#endif\n"
        "gl_FragColor = colorV * alpha;\n"

            "}";
    fshader->compileSourceCode(fsrc);

    glProgram = new QOpenGLShaderProgram;
    glProgram->addShader(vshader);
    glProgram->addShader(fshader);
    glProgram->link();

    glGenTextures(1, &glTexture);

    loadImage();
}

// Draws a spiral
void qglViewerTest::draw() {
  const float nbSteps = 200.0;

  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //glEnable(GL_CULL_FACE); // if enabled, faces are not visible when viewing the face in the same direction as its normal

  glDepthFunc(GL_LESS);     // The Type Of Depth Test To Do
  glShadeModel(GL_SMOOTH);  // Enables Smooth Color Shading

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glLineWidth(2.0);
  glPointSize(2);

  glEnable(GL_POINT_SPRITE);
  glEnable(GL_PROGRAM_POINT_SIZE);

  // Enable GL textures
  glEnable(GL_TEXTURE_2D);

  // Nice texture coordinate interpolation
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  // Display the quad
  glBindTexture(GL_TEXTURE_2D, glTexture);
  glBegin(GL_QUADS);
  glNormal3f(0.0, 0.0, 1.0);
  glColor3f(1.0, 1.0, 1.0); // reset color
  glTexCoord2f(0.0, 1.0 - v_max);
  glVertex3f(-u_max * ratio, -v_max, 0.0f);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-u_max * ratio, v_max, 0.0f);
  glTexCoord2f(u_max, 1.0);
  glVertex3f(u_max * ratio, v_max, 0.0f);
  glTexCoord2f(u_max, 1.0 - v_max);
  glVertex3f(u_max * ratio, -v_max, 0.0f);
  glEnd();
  glBindTexture(GL_TEXTURE_2D, 0);

  glBegin(GL_QUAD_STRIP);
  for (int i = 0; i < nbSteps; ++i) {
    const float ratio = i / nbSteps;
    const float angle = 21.0 * ratio;
    const float c = cos(angle);
    const float s = sin(angle);
    const float r1 = 1.0 - 0.8f * ratio;
    const float r2 = 0.8f - 0.8f * ratio;
    const float alt = ratio - 0.5f;
    const float nor = 0.5f;
    const float up = sqrt(1.0 - nor * nor);
    glColor3f(1.0 - ratio, 0.2f, ratio);
    glNormal3f(nor * c, up, nor * s);
    glVertex3f(r1 * c, alt, r1 * s);
    glVertex3f(r2 * c, alt + 0.05f, r2 * s);
  }
  glEnd();

  glProgram->bind();
  glProgram->setUniformValue("pointSize", GLfloat(4));
  glBegin(GL_POINTS);
  glColor4f(1.0, 1.0, 1.0, 1.0);
  glVertex3f(2.0, 2.0, 1.0);
  glVertex3f(2.0, 1.0, 1.0);
  glVertex3f(2.0, 3.0, 1.0);
  glEnd();
  glProgram->release();

  if (selectedPointFound) {
      glPointSize(6);
      glBegin(GL_POINTS);
      glColor3f(1.0, 1.0, 0.0);
      glVertex3fv(selectedPoint);
      glEnd();
  }

  Eigen::Matrix4f HomogeneusAxis = Eigen::Matrix4f::Zero();
  HomogeneusAxis(0,0) = 0.5; // axis lengths
  HomogeneusAxis(1,1) = 0.5;
  HomogeneusAxis(2,2) = 0.5;
  HomogeneusAxis.block<1,4>(3,0) = Eigen::MatrixXf::Ones(1,4);
  Eigen::Affine3f frame = Eigen::Affine3f::Identity();
  frame.rotate(Eigen::AngleAxis<float>(20.0 * (M_PI / 180.0), Eigen::Vector3f(0,0,1))); // z-axis rotation
  frame.rotate(Eigen::AngleAxis<float>(20.0 * (M_PI / 180.0), Eigen::Vector3f(0,1,0))); // y-axis rotation
  frame.rotate(Eigen::AngleAxis<float>(20.0 * (M_PI / 180.0), Eigen::Vector3f(1,0,0))); // x-axis rotation
  Eigen::Matrix4f axes = frame * HomogeneusAxis;
  const GLfloat vertices[] = { frame(0,3),frame(1,3),frame(2,3), axes(0,0),axes(1,0),axes(2,0), frame(0,3),frame(1,3),frame(2,3), axes(0,1),axes(1,1),axes(2,1), frame(0,3),frame(1,3),frame(2,3), axes(0,2),axes(1,2),axes(2,2), };
  const GLfloat colors[]  = { 1,0,0, 1,0,0, 0,1,0, 0,1,0, 0,0,1, 0,0,1 }; // Red-Green-Blue

  glLineWidth(2.0f);
  /*glColorPointer(3, GL_FLOAT, sizeof(float)*3, colors);
  glEnableClientState(GL_COLOR_ARRAY);
  glVertexPointer(3, GL_FLOAT, sizeof(float)*3, vertices);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDrawArrays(GL_LINES, 0, 6); // 6 vertices to create 3 lines
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);*/
  glBegin(GL_LINES);
  for (int l = 0; l < 3; l++) {
      glColor3f(colors[6*l + 0], colors[6*l + 1], colors[6*l + 2]);
      glVertex3f(vertices[6*l + 0], vertices[6*l + 1], vertices[6*l + 2]);

      glColor3f(colors[6*l + 3], colors[6*l + 4], colors[6*l + 5]);
      glVertex3f(vertices[6*l + 3], vertices[6*l + 4], vertices[6*l + 5]);
  }
  glEnd();

  // Render text in the end
  glColor3f(1.0, 1.0, 1.0);
  renderText(-2, -2, -0.5, "This is a test", QFont("Arial", 12));
}



void qglViewerTest::loadImage() {
/*
#if QT_VERSION < 0x040000
  QString name = QFileDialog::getOpenFileName(
      ".", "Images (*.png *.xpm *.jpg)", this, "Choose", "Select an image");
#else
  QString name = QFileDialog::getOpenFileName(this, "Select an image", ".",
                                              "Images (*.png *.xpm *.jpg)");
#endif
*/

  QString name = QString("/home/thomas/Downloads/libQGLViewer-2.7.2/examples/contribs/textureViewer/qglviewer.logo.png");

  // In case of Cancel
  if (name.isEmpty())
    return;

  QImage img(name);

  if (img.isNull()) {
    qWarning("Unable to load file, unsupported file format");
    return;
  }

#if QT_VERSION < 0x040000
  qWarning("Loading %s, %dx%d pixels", name.latin1(), img.width(),
           img.height());
#else
  qWarning("Loading %s, %dx%d pixels", name.toLatin1().constData(), img.width(),
           img.height());
#endif

  // 1E-3 needed. Just try with width=128 and see !
  int newWidth = 1 << (int)(1 + log(img.width() - 1 + 1E-3) / log(2.0));
  int newHeight = 1 << (int)(1 + log(img.height() - 1 + 1E-3) / log(2.0));

  u_max = img.width() / (float)newWidth;
  v_max = img.height() / (float)newHeight;

  if ((img.width() != newWidth) || (img.height() != newHeight)) {
    qWarning("Image size set to %dx%d pixels", newWidth, newHeight);
    img = img.copy(0, 0, newWidth, newHeight);
  }

  ratio = newWidth / float(newHeight);

  QImage glImg = QGLWidget::convertToGLFormat(img); // flipped 32bit RGBA

  // Bind the img texture...
  glBindTexture(GL_TEXTURE_2D, glTexture);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, 4, glImg.width(), glImg.height(), 0, GL_RGBA,
               GL_UNSIGNED_BYTE, glImg.bits());
  glBindTexture(GL_TEXTURE_2D, 0);
}

void qglViewerTest::keyPressEvent(QKeyEvent *e) {
  switch (e->key()) {
  case Qt::Key_L:
    loadImage();
    update();
    break;
  default:
    QGLViewer::keyPressEvent(e);
  }
}

/*
QString qglViewerTest::helpString() const {
  QString text("<h2>S i m p l e V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can move around, zoom and translate with the "
          "three mouse buttons. ";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}*/

/// Remove the default tabs of the help modal widget
void qglViewerTest::help()
{
    QGLViewer::help();
    this->helpWidget()->removeTab(3);
    this->helpWidget()->removeTab(0);
}

void qglViewerTest::mousePressEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton && e->modifiers() == Qt::ControlModifier) {
        QGLViewer::makeCurrent();
        selectedPoint = camera()->pointUnderPixel(e->pos(), selectedPointFound);
        qDebug() << selectedPointFound << "," << selectedPoint.x << "," << selectedPoint.y << "," << selectedPoint.z << "\n";
        //QGLViewer::mousePressEvent(e);
        update();
    }
    else if(e->button() == Qt::LeftButton && e->modifiers() == Qt::AltModifier) {
        QGLViewer::makeCurrent();
        if(camera()->setPivotPointFromPixel(e->pos())){
            camera()->setSceneCenter( camera()->pivotPoint() );
            setVisualHintsMask(1, 500);
        }
        QMouseEvent e_noModifier(e->type(), e->localPos(), e->windowPos(), e->screenPos(),
                                 e->button(), e->buttons(), Qt::NoModifier, e->source());
        QGLViewer::mousePressEvent(&e_noModifier);
    }
    else {
        QGLViewer::mousePressEvent(e);
    }
}

void qglViewerTest::mouseDoubleClickEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton && e->modifiers() == Qt::NoModifier)
    {
        /// Modified version of "RAP_FROM_PIXEL"
        QGLViewer::makeCurrent();
        selectedPoint = camera()->pointUnderPixel(e->pos(), selectedPointFound);
        qDebug() << selectedPointFound << "," << selectedPoint.x << "," << selectedPoint.y << "," << selectedPoint.z << "\n";
        if(!camera()->setPivotPointFromPixel(e->pos())){
            // std::cout << "failed" << std::endl;
            return; // do nothing
        }
        camera()->setSceneCenter( camera()->pivotPoint() );
        /// Stolen from "centerScene"
        camera()->frame()->projectOnLine(sceneCenter(), camera()->viewDirection());
        setVisualHintsMask(1);
        update();
    } else {
        /// Forward anything else to superclass
        QGLViewer::mouseDoubleClickEvent(e);
    }
}

void qglViewerTest::mouseMoveEvent(QMouseEvent* e)
{
    if(e->buttons() == Qt::NoButton && e->modifiers() == Qt::ControlModifier)
    {
        /// Modified version of "RAP_FROM_PIXEL"
        QGLViewer::makeCurrent();
        selectedPoint = camera()->pointUnderPixel(e->pos(), selectedPointFound);
        qDebug() << selectedPointFound << "," << selectedPoint.x << "," << selectedPoint.y << "," << selectedPoint.z << "\n";
        update();
    } else {
        /// Forward anything else to superclass
        QGLViewer::mouseMoveEvent(e);
    }
}

void qglViewerTest::setup_modelview(qglviewer::Camera* camera, QGLShaderProgram& program_)
{
    ///--- Fetch matrixes from trackball
    Eigen::Matrix4f MVP;
    camera->getModelViewProjectionMatrix(MVP.data());
    MVP.transposeInPlace();
    Eigen::Matrix4f MV;
    camera->getModelViewMatrix(MV.data());
    MV.transposeInPlace();

    // std::cout << "MVP:\n" << MVP << std::endl;
    // std::cout << "MV:\n" << MV << std::endl;

    ///--- Set shader variables
    program_.setUniformValue (program_.uniformLocation ("MVP"), QMatrix4x4(MVP.data()));
    program_.setUniformValue (program_.uniformLocation ("MV"), QMatrix4x4(MV.data()));
}
