#ifndef QGLVisualizer_H_
#define QGLVisualizer_H_

#include <QGLViewer/qglviewer.h>

#include <QMouseEvent>
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QGLShaderProgram>
#include <QGLViewer/qglviewer.h>
#include <QMatrix4x4>
#include <QVector3D>
#include <QTabWidget>
#include <QPainter>

#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>

#include <stdio.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "realsense/realsense.h"

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

#define MAIN_AXIS_LENGTH 1.0f
#define FRAME_AXIS_LENGTH 0.2f
#define TRAJECTORY_AXIS_LENGTH 0.1f
#define FRAME_AXIS_WIDTH 4.0f
#define AXIS_FONT_SIZE	16.0f
#define POINT_SIZE_NORMAL   1
#define POINT_SIZE_HIGHLIGHTED  3

// From https://stackoverflow.com/questions/13990788/how-do-i-use-a-vertex-buffer-object-with-a-vertex-array-in-opengl
#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + (bytes))

class QGLVisualizer : public QGLViewer {
public:
    QGLVisualizer(QWidget *parent = nullptr);

    ~QGLVisualizer();

protected:
    virtual void draw();
    //virtual void init();
    //virtual QString helpString() const;
    virtual void initializeGL();

    virtual void help();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void keyReleaseEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseDoubleClickEvent(QMouseEvent* e);
    virtual void mouseMoveEvent(QMouseEvent* e);

public:
    struct EIGEN_ALIGN16 color_t {
        color_t() : r(0.0f), g(0.0f), b(0.0f), dummy(0.0f) {};
        color_t(float red, float green, float blue) : r(red), g(green), b(blue), dummy(0.0f) {};
        float r;
        float g;
        float b;
        float dummy;
    };

    void OpenPCD(const std::string &pcd_file);

    template <typename PointT>
    float EstimateLongestAxisFromOrientedBoundingBox(const pcl::PointCloud<PointT> &point_cloud_ptr);

    void AdjustToScene(double sceneRadius);

    void AddPointcloud(const std::string pointcloud_name, uint8_t size = POINT_SIZE_NORMAL);
    void AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, uint8_t size = POINT_SIZE_NORMAL);
    void AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &new_pc, uint8_t size = POINT_SIZE_NORMAL, color_t color = color_t(1.0f, 1.0f, 1.0f));
    void AddDensePointcloud(const std::string pointcloud_name);

    void UpdatePointcloud(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc);
    void UpdatePointcloud(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_pc, color_t color = color_t(1.0f, 1.0f, 1.0f));
    void UpdatePointcloudWithNormals(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, const pcl::PointCloud<pcl::Normal>::Ptr &new_pc_normals);

    void ChangePointSize(const std::string pointcloud_name, uint8_t point_size);

    void RemovePointcloud(const std::string pointcloud_name);

    void ClearPointclouds();

    void
    UpdatePointcloudWithAxes(const std::string pointcloud_name, const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &new_pc);

    void SetPointcloudVisibility(const std::string pointcloud_name, bool visible);

    void VisualizeNormals(void);

    void SetViewpoint(Eigen::Matrix<float, 4, 4> view);
    void SetViewpoint(Eigen::Vector3f RPY_deg, Eigen::Vector3f translation = Eigen::Matrix<float, 3, 1>::Zero());
    Eigen::Matrix<float, 4, 4> GetViewpoint(void);
    void SetDefaultViewpoint();    

    void AddFrame(Eigen::Transform<float, 3, Eigen::Affine> tfFrame, const std::string &frameName, float axesLength = FRAME_AXIS_LENGTH);

    void UpdateFrame(const std::string &frameName, Eigen::Transform<double, 3, Eigen::Affine> tfFrame);
    void UpdateFrame(const std::string &frameName, Eigen::Transform<float, 3, Eigen::Affine> tfFrame);

    void ClearFrames(void);

    void AddTrajectory(const std::string &trajectoryName);

    void UpdateTrajectory(const std::string &trajectoryName, const std::vector<Eigen::Matrix4f> &poses);

    void ClearTrajectories(void);

    /* Button handlers */
    typedef enum mouseButton_t : int {
        Left = Qt::LeftButton,
        Middle = Qt::MiddleButton,
        Right = Qt::RightButton
    } mouseButton_t;

    typedef enum keyboardButton_t : int {
        None = 0,
        Shift = Qt::ShiftModifier,
        Ctrl = Qt::ControlModifier,
        Alt = Qt::AltModifier
    } keyboardButton_t;

    typedef enum keyboardButtonGroup_t : int {
        Characters = 0,
        Numbers
    } keyboardButtonGroup_t;

    void RegisterMouseButtonHandler(mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& func, keyboardButton_t keyCondition = None);
    Eigen::Vector3d GetMouse3Dposition();
    void RegisterSingleHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func);
    void RegisterDrawingHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func);
    void RegisterKeyboardButtonHandler(keyboardButtonGroup_t group, const boost::function<void (char)>& func, keyboardButton_t keyCondition = None);

    typedef enum DockLocation_t {
        TopLeft = 0,
        TopCenter,
        TopRight,
        MidLeft,
        MidCenter,
        MidRight,
        BottomLeft,
        BottomCenter,
        BottomRight,
        Top,
        Mid,
        Bottom
    } DockLocation_t;

private:
    typedef struct uniform_color_t {
        bool enabled;
        float r;
        float g;
        float b;
    };
    typedef struct pointcloud_t {
        std::string name;
        uniform_color_t uniform_color;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc{0};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_color{0};
        pcl::PointCloud<pcl::Normal>::Ptr pc_normals{0};
        std::shared_ptr<QOpenGLBuffer> vbo;
        std::shared_ptr<QOpenGLBuffer> vbo_color;
        std::shared_ptr<QOpenGLBuffer> vbo_normals;
        bool visible;
        bool dense; // will use shader for height coloring
        uint8_t size;
        unsigned long numberOfPoints;
        Eigen::Matrix<float, 4, 1> centroid;
        Eigen::Matrix<float, 3, 3> covariance;
        Eigen::Matrix<float, 3, 3> eigenvectors;
        Eigen::Matrix<float, 3, 1> eigenvalues;
    } uppointcloud_t;

    typedef struct trajectory_t {
        std::string name;
        std::vector<Eigen::Matrix4f> poses;
    } trajectory_t;

    typedef struct frame_t {
        std::string name;
        Eigen::Matrix4f transform;
        float axes_length;
    } frame_t;

    struct find_pointcloud : std::unary_function<pointcloud_t, bool> {
        std::string name;

        find_pointcloud(std::string name) : name(name) {}

        bool operator()(const pointcloud_t &s) const {
            return s.name == name;
        }
    };

    struct find_frame : std::unary_function<frame_t, bool> {
        std::string name;

        find_frame(std::string name) : name(name) {}

        bool operator()(const frame_t &s) const {
            return s.name == name;
        }
    };

    struct find_trajectory : std::unary_function<trajectory_t, bool> {
        std::string name;

        find_trajectory(std::string name) : name(name) {}

        bool operator()(const trajectory_t &s) const {
            return s.name == name;
        }
    };

    std::unique_ptr<boost::thread> VisualizerThread; // using unique_ptr helps deleting the object when the thread terminates
    std::vector<pointcloud_t> Pointclouds;
    std::vector<std::shared_ptr<QOpenGLBuffer>> VBOForClearance;
    boost::mutex frame_mtx;
    std::shared_ptr<QOpenGLShaderProgram> shaderProgram{0};
    bool keyRpressed{false};

    const QFont glFont;

    Eigen::Transform<float, 3, Eigen::Affine> tfLiDAR2Body;
    Eigen::Matrix<float, 4, 4> defaultViewpoint;

    bool selectedPointHighlighted{false};
    qglviewer::Vec selectedPoint;

    std::vector<frame_t> Frames;
    std::vector<trajectory_t> Trajectories;

    void configureQGL();

    void AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc,
                       float R, float G, float B, uint8_t size, int ring);

    pointcloud_t *FindPointcloud(const std::string pointcloud_name);

    void DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length);

    void DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length_x, float axis_length_y,
              float axis_length_z);

    //GLuint UploadPointCloud(pcl::PointCloud<nutils::PCLVelodyne>::Ptr &pc);
    void DrawPointCloud(pointcloud_t &pc);

    pcl::PointCloud <pcl::Normal> *CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

    void UploadNormals(std::shared_ptr<QOpenGLBuffer> vbo, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, float normalScale = 1.0f);
    void UploadNormals(std::shared_ptr<QOpenGLBuffer> vbo, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, float normalScale = 1.0f);

    std::shared_ptr<QOpenGLShaderProgram> ConfigureShaders(const char* vShaderCode, const char* fShaderCode);

    void renderText(int x, int y, const QString &str, const QFont &font);
    void renderText(double x, double y, double z, const QString &str, const QFont &font);

    void restoreGLconfig();

    qglviewer::Vec GetPointUnderPixel(const QPoint &pixel, bool &found);
};

#endif
