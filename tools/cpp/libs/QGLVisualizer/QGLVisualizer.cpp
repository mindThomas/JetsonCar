#include "QGLVisualizer.h"

#include <stdio.h>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>

#if QT_VERSION >= 0x040000
#include <QKeyEvent>
#endif

// See the following repositories:
// - https://github.com/GillesDebunne/libQGLViewer
// - https://github.com/PointCloudLibrary/pcl

/* Visualize PCD file */
QGLVisualizer::QGLVisualizer(QWidget *parent)
  : QGLViewer(parent)
  , shaderProgram(0)
  , glFont("Arial", 12)
{
    configureQGL();
}

QGLVisualizer::~QGLVisualizer()
{
}

void QGLVisualizer::configureQGL()
{
    // Restore previous viewer state.
    //restoreStateFromFile();

    // Opens help window
    //help();

    // Disable QGLViewer's default "spin"
    camera()->frame()->setSpinningSensitivity(100);
    camera()->frame()->setRotationSensitivity(1.0);
    camera()->frame()->setTranslationSensitivity(1.0);

    // Adjust these if loading larger pointclouds
    camera()->setSceneRadius(10);
    camera()->setZNearCoefficient(0.001);
    camera()->setZClippingCoefficient(10);

    // Bindings @see QGLViewer::setDefaultMouseBindings()
    // Extra behavior in this->mouseDoubleClickEvent()
    {
        // Disable double click to align scene
        setMouseBinding(Qt::NoModifier, Qt::LeftButton, NO_CLICK_ACTION, true);   /// ALIGN_CAMERA
        setMouseBinding(Qt::ShiftModifier, Qt::RightButton, NO_CLICK_ACTION);     /// RAP_FROM_PIXEL
        setMouseBinding(Qt::NoModifier, Qt::MiddleButton, NO_CLICK_ACTION, true); /// ZOOM_TO_FIT
    }
    setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, CAMERA, SCREEN_ROTATE);

    // Disable options that give OpenGL4 troubles
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
    setTextIsEnabled(true);
    setGridIsDrawn(true);
}


void QGLVisualizer::initializeGL()
{
    QGLViewer::initializeGL();     //initializeOpenGLFunctions();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);

    const char *vertexShader =
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

    const char *fragmentShader =
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

    shaderProgram = ConfigureShaders(vertexShader, fragmentShader);
}

void QGLVisualizer::restoreGLconfig()
{
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glEnable(GL_CULL_FACE); // if enabled, faces are not visible when viewing the face in the same direction as its normal

    glDepthFunc(GL_LESS);     // The Type Of Depth Test To Do
    glShadeModel(GL_SMOOTH);  // Enables Smooth Color Shading

    // Ensure that blending is enabled for rendering text.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Enable GL textures
    glDisable(GL_TEXTURE_2D);

    // Nice texture coordinate interpolation
    //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void QGLVisualizer::draw()
{
    frame_mtx.lock();

    restoreGLconfig();

    for (auto const& frame : Frames) {        
        DrawFrame(frame.transform, frame.name, frame.axes_length);
    }

    if (selectedPointHighlighted) {        
        // Draw highlighted point
        glPointSize(6);
        glBegin(GL_POINTS);
        glColor3f(1.0, 1.0, 0.0);
        glVertex3fv(selectedPoint);
        glEnd();
    }

    for (auto const& trajectory : Trajectories) {
        for (unsigned int i = 0; i < trajectory.poses.size(); i++) {            
            if (i == trajectory.poses.size()-1) // end frame
                DrawFrame(trajectory.poses[i], trajectory.name, 2*TRAJECTORY_AXIS_LENGTH);
            else
                DrawFrame(trajectory.poses[i], "", TRAJECTORY_AXIS_LENGTH);
        }
    }

    for (auto& pointcloud : Pointclouds) {        
        DrawPointCloud(pointcloud);
        if (pointcloud.eigenvalues.prod() > 0) {
            Eigen::Matrix4f axes = Eigen::Matrix4f::Zero();
            // swap order to have largest eigenvalue as x-axis
            axes.block<3,1>(0,0) = pointcloud.eigenvectors.block<3,1>(0,2);
            axes.block<3,1>(0,1) = pointcloud.eigenvectors.block<3,1>(0,1);
            axes.block<3,1>(0,2) = pointcloud.eigenvectors.block<3,1>(0,0);
            axes.block<4,1>(0,3) = pointcloud.centroid;
            // eigenvalue corresponds to sigma^2 - show axis length as 3*sigma
            DrawFrame(axes, "Cov{" + pointcloud.name + "}", sqrtf(pointcloud.eigenvalues(2))*3, sqrtf(pointcloud.eigenvalues(1))*3, sqrtf(pointcloud.eigenvalues(0))*3);
        }
    }    

    while (VBOForClearance.size()) {
        auto vbo = VBOForClearance.back();
        if (vbo->isCreated()) vbo->destroy(); // glDeleteBuffers(1, &(vbo)); // delete OpenGL buffer allocated to pointcloud
        VBOForClearance.pop_back();
    }

    frame_mtx.unlock();
}

void QGLVisualizer::help()
{
    QGLViewer::help();

    // Remove the default tabs of the help modal widget
    this->helpWidget()->removeTab(3);
    this->helpWidget()->removeTab(0);
}

qglviewer::Vec QGLVisualizer::GetPointUnderPixel(const QPoint &pixel, bool &found)
{
    const static int hwin = 4;
    const int zl = (hwin*2+1);
    const int zsize = zl*zl;
    GLfloat zs[zsize];

#ifndef HAVE_GLES
    glReadBuffer(GL_FRONT);
    // Qt uses upper corner for its origin while GL uses the lower corner.
    glReadPixels(pixel.x()-hwin, camera()->screenHeight() - 1 - pixel.y()-hwin, zl, zl,
                 GL_DEPTH_COMPONENT, GL_FLOAT, zs);
#else
    std::fill(zs,zs+zsize, 1);
#endif
    GLfloat mindepth = *(std::min_element(zs,zs+zsize));

    found = static_cast<double>(mindepth)< 1.0;

    qglviewer::Vec point(pixel.x(), pixel.y(), static_cast<double>(mindepth));
    point = camera()->unprojectedCoordinatesOf(point);

    return point;
}

void QGLVisualizer::mouseMoveEvent(QMouseEvent* e)
{    
    bool selectedPointFound = false;
    if (e->buttons() == Qt::NoButton && e->modifiers() == Qt::ControlModifier) {
        QGLViewer::makeCurrent();
        //selectedPoint = camera()->pointUnderPixel(e->pos(), selectedPointFound);
        selectedPoint = GetPointUnderPixel(e->pos(), selectedPointFound);
        //qDebug() << selectedPointFound << "," << selectedPoint.x << "," << selectedPoint.y << "," << selectedPoint.z << "\n";
        //QGLViewer::mousePressEvent(e);

        if (selectedPointFound) {
            selectedPointHighlighted = selectedPointFound;
            update();
        }
    }
    else {
            QGLViewer::mouseMoveEvent(e); // Forward anything else to superclass
    }

    /*if (selectedPointHighlighted != selectedPointFound) {
        selectedPointHighlighted = selectedPointFound;
        QGLViewer::makeCurrent();
        update();
    }*/
}

void QGLVisualizer::mousePressEvent(QMouseEvent* e)
{
    if ((e->buttons() == Qt::LeftButton || e->buttons() == Qt::RightButton) && e->modifiers() == Qt::NoModifier && !keyRpressed) {
        // Automatically change the pivot point to the current point at which rotation or movement is started (mouse button is pressed)
        bool foundPoint;
        QGLViewer::makeCurrent();
        qglviewer::Vec point = GetPointUnderPixel(e->pos(), foundPoint);
        if (foundPoint) {
            camera()->setPivotPoint(point);
            camera()->setSceneCenter( camera()->pivotPoint() );
        }
        QGLViewer::mousePressEvent(e); // Forward anything else to superclass
    }
    else if(e->button() == Qt::LeftButton && e->modifiers() == Qt::ControlModifier) {
        if (selectedPointHighlighted) {
            qDebug() << selectedPoint.x << "," << selectedPoint.y << "," << selectedPoint.z << "\n";
        }
    }
    else if(e->button() == Qt::LeftButton && e->modifiers() == Qt::AltModifier) {
        QGLViewer::makeCurrent();
        /*if(camera()->setPivotPointFromPixel(e->pos())){
            camera()->setSceneCenter( camera()->pivotPoint() );
            setVisualHintsMask(1, 500);
        }*/
        bool foundPoint;
        qglviewer::Vec point = GetPointUnderPixel(e->pos(), foundPoint);
        if (foundPoint) {
            camera()->setPivotPoint(point);
            camera()->setSceneCenter( camera()->pivotPoint() );
            setVisualHintsMask(1, 500);
        }
        QMouseEvent e_noModifier(e->type(), e->localPos(), e->windowPos(), e->screenPos(),
                                 e->button(), e->buttons(), Qt::NoModifier, e->source());
        QGLViewer::mousePressEvent(&e_noModifier);
    }
    else {
        QGLViewer::mousePressEvent(e); // Forward anything else to superclass
    }
}

void QGLVisualizer::mouseDoubleClickEvent(QMouseEvent* e)
{
    if(e->button() == Qt::LeftButton && e->modifiers() == Qt::NoModifier)
    {
        QGLViewer::makeCurrent();
        if(!camera()->setPivotPointFromPixel(e->pos())) {
            return; // do nothing
        }
        camera()->setSceneCenter( camera()->pivotPoint() );
        camera()->frame()->projectOnLine(sceneCenter(), camera()->viewDirection());
        setVisualHintsMask(1);
        update();
    } else {
        QGLViewer::mouseDoubleClickEvent(e); // Forward anything else to superclass
    }
}

void QGLVisualizer::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_R) {
        keyRpressed = true;
    }
    QGLViewer::keyPressEvent(e);
}


void QGLVisualizer::keyReleaseEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_R) {
        keyRpressed = false;
    }
    QGLViewer::keyReleaseEvent(e);
}


/*
void QGLVisualizer::randomImageData(unsigned char * imageArray, int size)
{
    for(int i = 0 ; i < size;i++) {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}
*/

void QGLVisualizer::OpenPCD(const std::string & pcd_file)
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

    printf("Opening Point Cloud example file\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file.c_str(), *pc) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file. \n");
        return;
    }

    AddPointcloud(basename(pcd_file.c_str()), pc);
}

void QGLVisualizer::AdjustToScene(double sceneRadius)
{
    setSceneRadius(sceneRadius);
    showEntireScene();
}

template float QGLVisualizer::EstimateLongestAxisFromOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB> &pcl);
template float QGLVisualizer::EstimateLongestAxisFromOrientedBoundingBox(const pcl::PointCloud<pcl::PointXYZ> &pcl);

template <typename PointT>
float QGLVisualizer::EstimateLongestAxisFromOrientedBoundingBox(const pcl::PointCloud<PointT> &pcl)
{
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(pcl, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(pcl, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointT> cPoints;
    pcl::transformPointCloud(pcl, cPoints, p2w);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    return std::max(std::max(
            std::max(std::fabs(min_pt.x + centroid[0]), std::fabs(max_pt.x + centroid[0])),
            std::max(std::fabs(min_pt.y + centroid[1]), std::fabs(max_pt.y + centroid[1]))),
            std::max(std::fabs(min_pt.z + centroid[2]), std::fabs(max_pt.z + centroid[2]))
    );
}

QGLVisualizer::pointcloud_t * QGLVisualizer::FindPointcloud(const std::string pointcloud_name)
{
    //std::vector<Sensor>::const_iterator it = std::find_if(sensors.begin(), sensors.end(), boost::bind(&Sensor::name, _1) == sensor_name);
    std::vector<QGLVisualizer::pointcloud_t>::const_iterator it = std::find_if(Pointclouds.begin(), Pointclouds.end(), find_pointcloud(pointcloud_name));
    if (it != Pointclouds.end())
        return (QGLVisualizer::pointcloud_t *)&*it;
    else
        return 0;
}

void QGLVisualizer::AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, uint8_t size)
{
    AddPointcloud(pointcloud_name, size);
    UpdatePointcloud(pointcloud_name, new_pc);
}

void QGLVisualizer::AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &new_pc, uint8_t size, color_t color)
{
    AddPointcloud(pointcloud_name, size);
    UpdatePointcloud(pointcloud_name, new_pc, color);
}

void QGLVisualizer::AddPointcloud(const std::string pointcloud_name, uint8_t size) // add empty pointcloud
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function

    QGLVisualizer::pointcloud_t * existPtr = FindPointcloud(pointcloud_name);
    if (existPtr) {
        throw std::runtime_error("Pointcloud name already exists");
    }

    QGLVisualizer::pointcloud_t pointcloud;
    pointcloud.name = pointcloud_name;
    pointcloud.pc.reset();
    pointcloud.uniform_color.enabled = true;
    pointcloud.uniform_color.r = 1.0f;
    pointcloud.uniform_color.g = 1.0f;
    pointcloud.uniform_color.b = 1.0f;
    pointcloud.pc_color.reset();
    pointcloud.pc_normals.reset();
    pointcloud.vbo = std::make_shared<QOpenGLBuffer>();
    pointcloud.vbo_color = std::make_shared<QOpenGLBuffer>();
    pointcloud.vbo_normals = std::make_shared<QOpenGLBuffer>();
    pointcloud.size = size;
    pointcloud.numberOfPoints = 0;
    pointcloud.dense = false;
    pointcloud.visible = true;
    pointcloud.eigenvalues.setZero();
    pointcloud.eigenvectors.setZero();
    Pointclouds.push_back(pointcloud);
}

void QGLVisualizer::AddDensePointcloud(const std::string pointcloud_name)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    AddPointcloud(pointcloud_name, POINT_SIZE_NORMAL); // white default color with normal size
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud)
        pointcloud->dense = true;
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::UpdatePointcloud(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->uniform_color.enabled = false;
        pointcloud->pc_color = new_pc;
        frame_mtx.unlock();
        update();
    } else {
        frame_mtx.unlock();
    }
}

void QGLVisualizer::UpdatePointcloud(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_pc, color_t color)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->uniform_color.enabled = true;
        pointcloud->uniform_color.r = color.r;
        pointcloud->uniform_color.g = color.g;
        pointcloud->uniform_color.b = color.b;
        pointcloud->pc = new_pc;
        frame_mtx.unlock();
        update();
    } else {
        frame_mtx.unlock();
    }
}

void QGLVisualizer::UpdatePointcloudWithNormals(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, const pcl::PointCloud<pcl::Normal>::Ptr &new_pc_normals)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->pc_color = new_pc;
        pointcloud->pc_normals = new_pc_normals;
        frame_mtx.unlock();
        update();
    } else {
        frame_mtx.unlock();
    }
}

void QGLVisualizer::SetPointcloudVisibility(const std::string pointcloud_name, bool visible)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->visible = visible;
    }
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::UpdatePointcloudWithAxes(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc)
{
    UpdatePointcloud(pointcloud_name, new_pc);

    pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);

    if (pointcloud) {
        pcl::compute3DCentroid<pcl::PointXYZRGB, float>(*new_pc, pointcloud->centroid);
        unsigned int n = pcl::computeCovarianceMatrix<pcl::PointXYZRGB, float>(*new_pc, pointcloud->centroid,
                                                                                  pointcloud->covariance);
        pointcloud->covariance = (1. / (n - 1.)) * pointcloud->covariance;
        pcl::eigen33<Eigen::Matrix<float, 3, 3>, Eigen::Matrix<float, 3, 1>>(pointcloud->covariance,
                                                                             pointcloud->eigenvectors,
                                                                             pointcloud->eigenvalues);
        /*std::cout << "Covariance: " << std::endl << pointcloud->covariance << std::endl;
        std::cout << "Eigenvectors: " << std::endl << pointcloud->eigenvectors << std::endl;
        std::cout << "Eigenvalues: " << std::endl << pointcloud->eigenvalues << std::endl;*/
    }
}

void QGLVisualizer::ChangePointSize(const std::string pointcloud_name, uint8_t point_size)
{
    pointcloud_t *pointcloud = FindPointcloud(pointcloud_name);

    if (pointcloud) {
        pointcloud->size = point_size;
    }
}

void QGLVisualizer::RemovePointcloud(const std::string pointcloud_name)
{
    frame_mtx.lock();
    QGLVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->pc.reset();
        VBOForClearance.emplace_back(pointcloud->vbo);
        VBOForClearance.emplace_back(pointcloud->vbo_color);

        auto pointcloud_iterator = Pointclouds.begin() + std::distance(Pointclouds.data(), pointcloud); // an equivalent vector::iterator
        Pointclouds.erase(pointcloud_iterator);
    }
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::ClearPointclouds()
{
    frame_mtx.lock();
    for (auto &pointcloud : Pointclouds) {
        pointcloud.pc.reset();
        VBOForClearance.emplace_back(pointcloud.vbo);
        VBOForClearance.emplace_back(pointcloud.vbo_color);
    }
    Pointclouds.clear();
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::VisualizeNormals(void)
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to normals_gl during this function

    // Compute normals of Point cloud
    printf("Computing normals of Point cloud\n");
    //pcl::PointCloud<pcl::Normal>::Ptr pc_normals(CalculateNormals(pc));

    // Upload normals to GPU
    //normals_gl = UploadNormals(pc, pc_normals, 0.02);
}

void QGLVisualizer::SetViewpoint(Eigen::Matrix<float, 4, 4> view)
{
    Eigen::MatrixXd view_double = view.cast<double>();
    camera()->setFromModelViewMatrix(view_double.data());
    update();
}

void QGLVisualizer::SetViewpoint(Eigen::Vector3f RPY_deg, Eigen::Vector3f translation)
{
    auto rot = Eigen::AngleAxis<float>(deg2rad(RPY_deg(2)), Eigen::Vector3f(0,0,1)) *
               Eigen::AngleAxis<float>(deg2rad(RPY_deg(1)), Eigen::Vector3f(0,1,0)) *
               Eigen::AngleAxis<float>(deg2rad(RPY_deg(0)), Eigen::Vector3f(1,0,0));

    Eigen::Affine3f view = Eigen::Translation3f(translation) * rot;
    SetViewpoint(view.matrix());
}

void QGLVisualizer::SetDefaultViewpoint()
{
    SetViewpoint(defaultViewpoint);
}

Eigen::Matrix<float, 4, 4> QGLVisualizer::GetViewpoint(void)
{
    Eigen::Matrix<float, 4, 4> view;
    camera()->getModelViewMatrix(view.data()); // getModelViewProjectionMatrix
    return view;
}

void QGLVisualizer::DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length)
{
    DrawFrame(frame, frameName, axis_length, axis_length, axis_length);
}

void QGLVisualizer::DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length_x, float axis_length_y, float axis_length_z)
{
    Eigen::Matrix4f HomogeneusAxis = Eigen::Matrix4f::Zero();
    HomogeneusAxis(0,0) = axis_length_x;
    HomogeneusAxis(1,1) = axis_length_y;
    HomogeneusAxis(2,2) = axis_length_z;
    HomogeneusAxis.block<1,4>(3,0) = Eigen::MatrixXf::Ones(1,4);

    Eigen::Matrix4f axes = frame * HomogeneusAxis;
    const GLfloat vertices[] = { frame(0,3),frame(1,3),frame(2,3), axes(0,0),axes(1,0),axes(2,0), frame(0,3),frame(1,3),frame(2,3), axes(0,1),axes(1,1),axes(2,1), frame(0,3),frame(1,3),frame(2,3), axes(0,2),axes(1,2),axes(2,2), };
    const GLfloat colors[]  = { 1,0,0, 1,0,0, 0,1,0, 0,1,0, 0,0,1, 0,0,1 }; // Red-Green-Blue

    glLineWidth(FRAME_AXIS_WIDTH);
    glColorPointer(3, GL_FLOAT, sizeof(float)*3, colors);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(float)*3, vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINES, 0, 6); // 6 vertices to create 3 lines
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    /*glBegin(GL_LINES);
    for (int l = 0; l < 3; l++) {
        glColor3f(colors[6*l + 0], colors[6*l + 1], colors[6*l + 2]);
        glVertex3f(vertices[6*l + 0], vertices[6*l + 1], vertices[6*l + 2]);

        glColor3f(colors[6*l + 3], colors[6*l + 4], colors[6*l + 5]);
        glVertex3f(vertices[6*l + 3], vertices[6*l + 4], vertices[6*l + 5]);
    }
    glEnd();*/

    if (frameName.length() > 0) {
        glColor3f(1.0, 1.0, 1.0);
        renderText(frame(0, 3), frame(1, 3), frame(2, 3), QString::fromStdString(frameName), glFont);
    }
}

void QGLVisualizer::renderText(int x, int y, const QString &str, const QFont &font)
{
  // Retrieve last OpenGL color to use as a font color
  GLdouble glColor[4];
  glGetDoublev(GL_CURRENT_COLOR, glColor);
  QColor fontColor = QColor(255 * glColor[0], 255 * glColor[1],
                            255 * glColor[2], 255 * glColor[3]);

  // Save current OpenGL state
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // Render text
  QPainter painter(this);
  painter.setPen(fontColor);
  painter.setFont(font);
  painter.drawText(x, y, str);
  painter.end();

  // Restore OpenGL state
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
}

void QGLVisualizer::renderText(double x, double y, double z, const QString &str, const QFont &font)
{
  const qglviewer::Vec proj = camera()->projectedCoordinatesOf(qglviewer::Vec(x, y, z));
  renderText(proj.x, proj.y, str, font);
}

pcl::PointCloud<pcl::Normal> * QGLVisualizer::CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    // See http://pointclouds.org/documentation/tutorials/normal_estimation.php
    // Initialize a normal estimation object
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // Pass the cloud to the estimator
    ne.setInputCloud(pc);

    // Set viewpoint from where the normals are calculated
    ne.setViewPoint(0, 0, 0);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal> * cloud_normals = new pcl::PointCloud<pcl::Normal>;

    // Use all neighbors in a sphere of radius 3 cm
    ne.setRadiusSearch(0.3);
    // ne.setKSearch(5); // use the 5 closest points

    // Compute the features
    ne.compute(*cloud_normals);

    return cloud_normals;
}

/*pangolin::GlBuffer * QGLVisualizer::UploadPointCloud(pcl::PointCloud<nutils::PCLVelodyne>::Ptr &pc)
{
    float pointArray[3*pc->points.size()];
    for (int i = 0; i < pc->points.size(); i++) memcpy((char*)(&pointArray[3*i]), (char*)pc->points[i].data, 3*sizeof(float));

    //pangolin::GlBuffer * glBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, pc->points.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
    GLuint bo;

    glGenBuffers(1, &bo);
    glBindBuffer(GL_ARRAY_BUFFER, bo);
    glBufferData(GL_ARRAY_BUFFER, pc->points.size()*3*sizeof(float), 0, GL_STATIC_DRAW);

    //glBuf->Upload(pointArray, 3*sizeof(float)*pc->points.size());
    glBufferSubData(GL_ARRAY_BUFFER, 0, pc->points.size()*3*sizeof(float), pointArray);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return bo;
}*/

void QGLVisualizer::DrawPointCloud(pointcloud_t &pointcloud)
//void QGLVisualizer::DrawPointCloud(GLuint glBuf, GLuint pointcloud_size)
{
    if (pointcloud.pc_color) { // pointcloud has been changed
        pointcloud.numberOfPoints = pointcloud.pc_color->points.size();
        std::vector<color_t> colors;
        for (auto pt : pointcloud.pc_color->points)
        {
            color_t ptColor;
            ptColor.r = (float)pt.r / 255.0f;
            ptColor.g = (float)pt.g / 255.0f;
            ptColor.b = (float)pt.b / 255.0f;
            colors.push_back(ptColor);
        }

        if (pointcloud.pc_normals)
            if (pointcloud.pc_normals->size() != pointcloud.pc_color->size())
                pointcloud.pc_normals.reset();


        // More optimal way of uploading data directly from pointcloud object
        /* Upload data to GPU buffer */
        if (pointcloud.vbo->isCreated()) pointcloud.vbo->destroy(); // delete previous buffer
        if (pointcloud.vbo_color->isCreated()) pointcloud.vbo_color->destroy(); // delete previous buffer
        if (pointcloud.vbo_normals->isCreated()) pointcloud.vbo_normals->destroy(); // delete previous buffer

        if (pointcloud.numberOfPoints > 0) {
            pointcloud.vbo->create(); // glGenBuffers(1, &(pointcloud.vbo)); // generate 1 buffer
            pointcloud.vbo->bind();   // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
            pointcloud.vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
            pointcloud.vbo->allocate(pointcloud.pc_color->points.data(), pointcloud.pc_color->points.size() * sizeof(pcl::PointXYZRGB)); // glBufferData(GL_ARRAY_BUFFER, pointcloud.pc->points.size() * sizeof(pcl::PointXYZRGB), pointcloud.pc->points.data(), GL_STATIC_DRAW);
            pointcloud.vbo->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);

            /* Colored points : https://github.com/mp3guy/Kintinuous/blob/master/src/utils/PangoCloud.h */
            pointcloud.vbo_color->create(); // glGenBuffers(1, &(pointcloud.vbo_color)); // generate 1 buffer
            pointcloud.vbo_color->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_color);
            pointcloud.vbo_color->setUsagePattern(QOpenGLBuffer::StaticDraw);
            pointcloud.vbo_color->allocate(colors.data(), colors.size() * sizeof(color_t)); // glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(color_t), colors.data(), GL_STATIC_DRAW);
            pointcloud.vbo_color->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);

            if (pointcloud.pc_normals) {
                /* Normal upload */
                pointcloud.vbo_normals->create(); // glGenBuffers(1, &(pointcloud.vbo_normals)); // generate 1 buffer
                pointcloud.vbo_normals->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_normals);
                UploadNormals(pointcloud.vbo_normals, pointcloud.pc_color, pointcloud.pc_normals);
                pointcloud.vbo_normals->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }

        pointcloud.pc_color.reset();
    }

    if (pointcloud.pc) { // pointcloud has been changed
        pointcloud.numberOfPoints = pointcloud.pc->points.size();

        if (pointcloud.pc_normals)
            if (pointcloud.pc_normals->size() != pointcloud.pc->size())
                pointcloud.pc_normals.reset();


        // More optimal way of uploading data directly from pointcloud object
        /* Upload data to GPU buffer */
        if (pointcloud.vbo->isCreated()) pointcloud.vbo->destroy(); // delete previous buffer
        if (pointcloud.vbo_normals->isCreated()) pointcloud.vbo_normals->destroy(); // delete previous buffer

        if (pointcloud.numberOfPoints > 0) {
            pointcloud.vbo->create(); // glGenBuffers(1, &(pointcloud.vbo)); // generate 1 buffer
            pointcloud.vbo->bind();   // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
            pointcloud.vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
            pointcloud.vbo->allocate(pointcloud.pc->points.data(), pointcloud.pc->points.size() * sizeof(pcl::PointXYZ)); // glBufferData(GL_ARRAY_BUFFER, pointcloud.pc->points.size() * sizeof(pcl::PointXYZRGB), pointcloud.pc->points.data(), GL_STATIC_DRAW);
            pointcloud.vbo->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);

            if (pointcloud.pc_normals) {
                /* Normal upload */
                pointcloud.vbo_normals->create(); // glGenBuffers(1, &(pointcloud.vbo_normals)); // generate 1 buffer
                pointcloud.vbo_normals->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_normals);
                UploadNormals(pointcloud.vbo_normals, pointcloud.pc, pointcloud.pc_normals);
                pointcloud.vbo_normals->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }

        pointcloud.pc.reset();
    }

    if (pointcloud.numberOfPoints == 0) return;
    if (!pointcloud.vbo || !pointcloud.vbo->isCreated()) return; // something went wrong, we don't have a VBO to display
    if (!pointcloud.uniform_color.enabled && (!pointcloud.vbo_color || !pointcloud.vbo_color->isCreated())) return; // something went wrong, we don't have a VBO to display
    if (!pointcloud.visible) return;    

    /* Render buffered point cloud */
    glPointSize(pointcloud.size);
    if (pointcloud.uniform_color.enabled) {
        glColor4f(pointcloud.uniform_color.r, pointcloud.uniform_color.g, pointcloud.uniform_color.b, 1);
    } else {
        glEnableClientState(GL_COLOR_ARRAY);
    }
    glEnableClientState(GL_VERTEX_ARRAY);
    pointcloud.vbo->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
    if (pointcloud.uniform_color.enabled) {
        glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZ), 0); // important to set stride
    } else {
        glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZRGB), 0); // important to set stride
    }
    pointcloud.vbo->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);
    if (!pointcloud.uniform_color.enabled) {
        pointcloud.vbo_color->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_color);
        glColorPointer(3, GL_FLOAT, sizeof(color_t), 0); // important to set stride
        pointcloud.vbo_color->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    /*if (shaderProgram) {
        shaderProgram->bind();
        shaderProgram->setUniformValue("pointSize", GLfloat(pointcloud.size));
    }*/
    glDrawArrays(GL_POINTS, 0, pointcloud.numberOfPoints);
    /*if (shaderProgram)
        shaderProgram->release();*/

    glDisableClientState(GL_VERTEX_ARRAY);
    if (!pointcloud.uniform_color.enabled)
        glDisableClientState(GL_COLOR_ARRAY);

    /* Draw normals */
    if (pointcloud.vbo_normals->isCreated()) {
        glEnableClientState(GL_VERTEX_ARRAY);

        glLineWidth(1.0f);
        pointcloud.vbo_normals->bind(); // glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_normals);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glColor4f(0, 0, 1, 1);
        glDrawArrays(GL_LINES, 0, 2*pointcloud.numberOfPoints);
        glDisableClientState(GL_VERTEX_ARRAY);

        pointcloud.vbo_normals->release(); // glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void QGLVisualizer::UploadNormals(std::shared_ptr<QOpenGLBuffer> vbo, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, float normalScale)
{
    float * pointArray = (float*)malloc(6*pc->points.size() * sizeof(float));
    unsigned long normalCount = 0;
    unsigned long j = 0;
    for (unsigned long i = 0; i < pc->points.size(); i++) {
        Eigen::Vector3f normal = pc_normals->points[i].getNormalVector3fMap();
        //float normal_length = normal.norm();
        Eigen::Vector3f normalized_normal = normal.normalized();

        if (!normal.hasNaN()) {
            //std::cout << "Normal " << i << std::endl << normalized_normal << std::endl << std::endl;

            //memcpy((char*)(&pointArray[6*i]), (char*)pc->points[i].data, 3*sizeof(float));
            pointArray[j + 0] = pc->points[i].x + 0.01*normalScale*normalized_normal[0];
            pointArray[j + 1] = pc->points[i].y + 0.01*normalScale*normalized_normal[1];
            pointArray[j + 2] = pc->points[i].z + 0.01*normalScale*normalized_normal[2];

            pointArray[j + 3] = pointArray[j + 0] + normalScale * normalized_normal[0];
            pointArray[j + 4] = pointArray[j + 1] + normalScale * normalized_normal[1];
            pointArray[j + 5] = pointArray[j + 2] + normalScale * normalized_normal[2];

            normalCount++;
            j += 6;
        }
    }

    /*pangolin::GlBuffer * glBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, 2*normalCount, GL_FLOAT, 3, GL_STATIC_DRAW);
    glBuf->Upload(pointArray, 6*sizeof(float)*normalCount);*/
    //glBufferData(GL_ARRAY_BUFFER, 6 * normalCount * sizeof(float), pointArray, GL_STATIC_DRAW);
    vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo->allocate(pointArray, 6 * normalCount * sizeof(float)); // glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(color_t), colors.data(), GL_STATIC_DRAW);
}

void QGLVisualizer::UploadNormals(std::shared_ptr<QOpenGLBuffer> vbo, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, float normalScale)
{
    float * pointArray = (float*)malloc(6*pc->points.size() * sizeof(float));
    unsigned long normalCount = 0;
    unsigned long j = 0;
    for (unsigned long i = 0; i < pc->points.size(); i++) {
        Eigen::Vector3f normal = pc_normals->points[i].getNormalVector3fMap();
        //float normal_length = normal.norm();
        Eigen::Vector3f normalized_normal = normal.normalized();

        if (!normal.hasNaN()) {
            //std::cout << "Normal " << i << std::endl << normalized_normal << std::endl << std::endl;

            //memcpy((char*)(&pointArray[6*i]), (char*)pc->points[i].data, 3*sizeof(float));
            pointArray[j + 0] = pc->points[i].x + 0.01*normalScale*normalized_normal[0];
            pointArray[j + 1] = pc->points[i].y + 0.01*normalScale*normalized_normal[1];
            pointArray[j + 2] = pc->points[i].z + 0.01*normalScale*normalized_normal[2];

            pointArray[j + 3] = pointArray[j + 0] + normalScale * normalized_normal[0];
            pointArray[j + 4] = pointArray[j + 1] + normalScale * normalized_normal[1];
            pointArray[j + 5] = pointArray[j + 2] + normalScale * normalized_normal[2];

            normalCount++;
            j += 6;
        }
    }

    /*pangolin::GlBuffer * glBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, 2*normalCount, GL_FLOAT, 3, GL_STATIC_DRAW);
    glBuf->Upload(pointArray, 6*sizeof(float)*normalCount);*/
    //glBufferData(GL_ARRAY_BUFFER, 6 * normalCount * sizeof(float), pointArray, GL_STATIC_DRAW);
    vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo->allocate(pointArray, 6 * normalCount * sizeof(float)); // glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(color_t), colors.data(), GL_STATIC_DRAW);
}

void QGLVisualizer::AddFrame(Eigen::Transform<float,3,Eigen::Affine> tfFrame, const std::string &frameName, float axesLength)
{
    frame_t frame;
    frame.name = frameName;
    frame.transform = tfFrame.matrix();
    frame.axes_length = axesLength;
    frame_mtx.lock();
    Frames.push_back(frame);
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::UpdateFrame(const std::string &frameName, Eigen::Transform<double,3,Eigen::Affine> tfFrame)
{
    frame_mtx.lock();
    std::vector<frame_t>::const_iterator it = std::find_if(Frames.begin(), Frames.end(), find_frame(frameName));
    if (it != Frames.end()) {
        frame_t *frame = (frame_t *) &*it;
        frame->transform = tfFrame.matrix().cast<float>();
    }
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::UpdateFrame(const std::string &frameName, Eigen::Transform<float,3,Eigen::Affine> tfFrame)
{
    frame_mtx.lock();
    std::vector<frame_t>::const_iterator it = std::find_if(Frames.begin(), Frames.end(), find_frame(frameName));
    if (it != Frames.end()) {
        frame_t *frame = (frame_t *) &*it;
        frame->transform = tfFrame.matrix();
    }
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::ClearFrames(void)
{
    frame_mtx.lock();
    Frames.clear();
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::AddTrajectory(const std::string &trajectoryName)
{
    trajectory_t trajectory;
    trajectory.name = trajectoryName;
    frame_mtx.lock();
    Trajectories.push_back(trajectory);
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::UpdateTrajectory(const std::string &trajectoryName, const std::vector<Eigen::Matrix4f> &poses)
{
    frame_mtx.lock();
    std::vector<trajectory_t>::const_iterator it = std::find_if(Trajectories.begin(), Trajectories.end(), find_trajectory(trajectoryName));
    if (it != Trajectories.end()) {
        trajectory_t *trajectory = (trajectory_t *) &*it;
        trajectory->poses = poses;
    }
    frame_mtx.unlock();
    update();
}

void QGLVisualizer::ClearTrajectories(void)
{
    frame_mtx.lock();
    Trajectories.clear();
    frame_mtx.unlock();
    update();
}


// https://learnopengl.com/Getting-started/Shaders
std::shared_ptr<QOpenGLShaderProgram> QGLVisualizer::ConfigureShaders(const char* vShaderCode, const char* fShaderCode)
{
    QOpenGLShader * vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    QOpenGLShader * fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);

    vshader->compileSourceCode(vShaderCode);
    fshader->compileSourceCode(fShaderCode);

    auto glProgram = std::make_shared<QOpenGLShaderProgram>();
    glProgram->addShader(vshader);
    glProgram->addShader(fshader);
    glProgram->link();

    return glProgram;
}

void QGLVisualizer::RegisterMouseButtonHandler(QGLVisualizer::mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& func, QGLVisualizer::keyboardButton_t keyCondition)
{

}

Eigen::Vector3d QGLVisualizer::GetMouse3Dposition()
{
    return Eigen::Vector3d(0,0,0);
}

void QGLVisualizer::RegisterSingleHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func)
{

}

void QGLVisualizer::RegisterDrawingHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func)
{

}

void QGLVisualizer::RegisterKeyboardButtonHandler(QGLVisualizer::keyboardButtonGroup_t group, const boost::function<void (char)>& func, QGLVisualizer::keyboardButton_t keyCondition)
{

}

void AddOBJmodel(const std::string model_name, std::string OBJfilePath) {
    // See http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
    // Implement something similar to Pointcloud visualization, thus make a vector of added objects

    // When drawing the elements you need to adjust the space in which the points are added by:
    //    glPushMatrix();
    //    glRotatef(angle, axes.x, axes.y, axes.z);   or     glMultMatrixf(r);
    //    glPopMatrix();
}
