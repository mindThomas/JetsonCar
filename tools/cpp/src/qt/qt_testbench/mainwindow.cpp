#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qglViewerTest * qglViewer = new qglViewerTest(this);
    qglVisualizer = new QGLVisualizer(this);
    qglVisualizer->AddFrame(Eigen::Affine3f::Identity(), "Origin");
    qglVisualizer->AddPointcloud("Loaded");

    ui->tabWidget->addTab(new ImuTab, "IMU");            
    ui->tabWidget->addTab(new TestTab, "Test");
    ui->tabWidget->addTab(new Window, "OpenGL");

    {
        Qt3DExtras::Qt3DWindow * view = new Qt3DExtras::Qt3DWindow;
        view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x000000)));

        Qt3DCore::QEntity *scene = createTestScene();

        // Camera
        Qt3DRender::QCamera *camera = view->camera();
        camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
        camera->setPosition(QVector3D(0, 0, 40.0f));
        camera->setViewCenter(QVector3D(0, 0, 0));

        // For camera controls
        Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(scene);
        camController->setLinearSpeed( 50.0f );
        camController->setLookSpeed( 180.0f );
        camController->setCamera(camera);

        view->setRootEntity(scene);
        QWidget *container = QWidget::createWindowContainer(view);
        ui->tabWidget->addTab(container, "Qt3D");
    }

    ui->tabWidget->addTab(qglViewer, "QGLViewer");
    ui->tabWidget->addTab(qglVisualizer, "QGLVisualizer");

    QTimer::singleShot(2000, this, [this]() {
        checkForReady();
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

Qt3DCore::QEntity* MainWindow::createTestScene()
{
    Qt3DCore::QEntity* root = new Qt3DCore::QEntity;
    Qt3DCore::QEntity* torus = new Qt3DCore::QEntity(root);

    Qt3DExtras::QTorusMesh* mesh = new Qt3DExtras::QTorusMesh;
    mesh->setRadius(5);
    mesh->setMinorRadius(1);
    mesh->setRings(100);
    mesh->setSlices(20);

    Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
//    transform->setScale3D(QVector3D(1.5, 1, 0.5));
    transform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 45.f ));

    //Qt3DRender::QMaterial* material = new Qt3DExtras::QPhongMaterial(root);
    Qt3DExtras::QPhongAlphaMaterial *material = new Qt3DExtras::QPhongAlphaMaterial();
    material->setAmbient(QColor("#576675"));
    material->setDiffuse(QColor("#5F6E7D"));
    material->setSpecular(QColor("#61707F"));
    material->setShininess(0.0f);
    material->setAlpha(1.0f);

    torus->addComponent(mesh);
    torus->addComponent(transform);
    torus->addComponent(material);

    return root;
}

pcl::PointXYZRGB MainWindow::point(float x, float y, float z, uint8_t R, uint8_t G, uint8_t B)
{
    pcl::PointXYZRGB pt(R, G, B);
    pt.x = x;
    pt.y = y;
    pt.z = z;
    return pt;
}

void MainWindow::checkForReady(void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl->push_back(point(0.0, 0.0, 0.0, 255, 255, 255));
    pcl->push_back(point(1.0, 0.0, 0.0, 255, 0, 0));
    pcl->push_back(point(0.0, 1.0, 0.0, 0, 255, 0));
    pcl->push_back(point(0.0, 0.0, 1.0, 0, 0, 255));
    qglVisualizer->AddPointcloud("Test", pcl, 10);
}

void MainWindow::on_actionOpen_PCD_file_triggered()
{
    QString FilePath = QFileDialog::getOpenFileName(this, "Select a pointcloud", ".", "Pointcloud (*.pcd)");

    if (FilePath.isEmpty()) {
        QMessageBox::critical( this, "Open PCD", QString("File not specified!"), QMessageBox::Ok );
        return;
    }

    if (!utils::FileExist(FilePath.toStdString())) {
        QMessageBox::critical( this, "Open PCD", QString("File does not exist {%1}!").arg(FilePath), QMessageBox::Ok );
        return;
    }

    if (utils::getFileExtension(FilePath.toStdString()).compare(".pcd")) {
        QMessageBox::critical( this, "Open PCD", QString("Incorrect file type %1 (should be .pcd)").arg(QString::fromStdString(utils::getFileExtension(FilePath.toStdString()))), QMessageBox::Ok );
        return;
    }

    auto pc_new = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto ret = pcl::io::loadPCDFile(FilePath.toStdString(), *pc_new);

    if (ret != 0) {
        QMessageBox::critical( this, "Open PCD", QString("Could not parse PCD file (unknown format)."), QMessageBox::Ok );
        return;
    }

    qglVisualizer->UpdatePointcloud("Loaded", pc_new);
    qglVisualizer->AdjustToScene(qglVisualizer->EstimateLongestAxisFromOrientedBoundingBox(*pc_new));
}
