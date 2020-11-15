#include "qtGLVisualizer.h"

#include <QTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <cmath>
#include <QPainter>

qtGLVisualizer::qtGLVisualizer(QWidget *parent) : QOpenGLWidget(parent)
{    
    // Multisampling
    QSurfaceFormat format;
    format.setSamples(4);
    setFormat(format);
}

qtGLVisualizer::~qtGLVisualizer()
{
    makeCurrent();
    doneCurrent();
}

void qtGLVisualizer::initializeGL()
{      
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
            "attribute highp vec4 vertex;\n"
            "attribute mediump vec4 texCoord;\n"
            "varying mediump vec4 texc;\n"
            "uniform mediump mat4 matrix;\n"
            "void main(void)\n"
            "{\n"
            "    gl_Position = matrix * vertex;\n"
            "    texc = texCoord;\n"
            "}\n";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
            "uniform sampler2D texture;\n"
            "varying mediump vec4 texc;\n"
            "void main(void)\n"
            "{\n"
            "    gl_FragColor = texture2D(texture, texc.st);\n"
            "}\n";
    fshader->compileSourceCode(fsrc);

    /*mProgram = new QOpenGLShaderProgram;
    mProgram->addShader(vshader);
    mProgram->addShader(fshader);
    mProgram->bindAttributeLocation("vertex", 0);
    mProgram->bindAttributeLocation("texCoord", 1);
    mProgram->link();

    mProgram->bind();
    mProgram->setUniformValue("texture", 0);*/
}

void qtGLVisualizer::paintGL()
{
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 m;
    double aspect = double(width()) / double(height() ? height() : 1);
    m.perspective(30.0 / qMin(aspect, 1.0), aspect, 0.5, 5.0);
    m.translate(0.0f, 0.0f, -1.2f);
    m.rotate(180, 0.0f, 1.0f, 0.0f);
    m.rotate(180, 0.0f, 0.0f, 1.0f);

    /*if (mUseQuaternions) {
        // TODO: Test if this works...
        m.rotate(QQuaternion(mQ0, mQ1, mQ2, mQ3));
    } else {
        m.rotate(mZRot, 0.0f, 1.0f, 0.0f);
        m.rotate(mYRot, 1.0f, 0.0f, 0.0f);
        m.rotate(mXRot, 0.0f, 0.0f, 1.0f);
    }
    m.scale(1.0, 0.2, 1.0);
    mProgram->setUniformValue("matrix", m);

    mProgram->enableAttributeArray(0);
    mProgram->enableAttributeArray(1);
    mProgram->setAttributeBuffer(0, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    mProgram->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

    for (int i = 0; i < 6; ++i) {
        mGLVisualizer[i]->bind();
        glDrawArrays(GL_TRIANGLE_FAN, i * 4, 4);
    }*/
}
