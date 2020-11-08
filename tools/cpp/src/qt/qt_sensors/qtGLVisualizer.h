#ifndef QT_GLVISUALIZER_H
#define QT_GLVISUALIZER_H

#include <QObject>
#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)
QT_FORWARD_DECLARE_CLASS(GLVisualizer)

namespace pangolin {
class OpenGlRenderState;
}

class qtGLVisualizer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit qtGLVisualizer(QWidget *parent = nullptr);
    ~qtGLVisualizer();

protected:
    void initializeGL();
    void paintGL();

private:
    GLVisualizer * visualizer;
    pangolin::OpenGlRenderState * s_cam;

};

#endif
