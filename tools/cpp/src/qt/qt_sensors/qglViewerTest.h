#ifndef QGLVIEWERTEST_H
#define QGLVIEWERTEST_H

#include <QGLViewer/qglviewer.h>

#include <Eigen/Dense>
#include <QMouseEvent>
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QGLShaderProgram>
#include <QGLViewer/qglviewer.h>
#include <QMatrix4x4>
#include <QVector3D>
#include <QTabWidget>

#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include <qfiledialog.h>
#include <qimage.h>

class qglViewerTest : public QGLViewer {
    public:
      qglViewerTest(QWidget *parent = nullptr);

      void loadImage();
      static void setup_modelview(qglviewer::Camera* camera, QGLShaderProgram& program_);

    protected:
      virtual void draw();
      //virtual void init();
      //virtual QString helpString() const;
      virtual void initializeGL();

      virtual void help();
      virtual void keyPressEvent(QKeyEvent *e);
      virtual void mousePressEvent(QMouseEvent* e);
      virtual void mouseDoubleClickEvent(QMouseEvent* e);
      virtual void mouseMoveEvent(QMouseEvent* e);

    private:
      float ratio, u_max, v_max;
      GLuint glTexture;

      QOpenGLShaderProgram * glProgram;

      bool selectedPointFound{false};
      qglviewer::Vec selectedPoint;
};
#endif
