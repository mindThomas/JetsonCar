#include "realsensetab.h"
#include "ui_realsensetab.h"

RealsenseTab::RealsenseTab(Realsense *realsense_, QWidget *parent) :
    realsense(realsense_),
    QWidget(parent),
    ui(new Ui::RealsenseTab)
{
    QGridLayout * gridLayout = new QGridLayout(this);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    QTabWidget * tabWidget = new QTabWidget(parent);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    gridLayout->addWidget(tabWidget, 0, 0, 1, 1);
    tabWidget->setCurrentIndex(-1);

    {
        QWidget * tab = new QWidget;
        ui->setupUi(tab);
        connect(ui->resetOriginBtn, SIGNAL(clicked()), SLOT(on_resetOriginBtn_clicked()));
        connect(ui->getViewBtn, SIGNAL(clicked()), SLOT(on_getViewBtn_clicked()));
        connect(ui->getOrientationBtn, SIGNAL(clicked()), SLOT(on_getOrientationBtn_clicked()));
        connect(ui->resetViewBtn, SIGNAL(clicked()), SLOT(on_resetViewBtn_clicked()));
        tabWidget->addTab(tab, "3D");
    }

    {
        QWidget * tab = new QWidget;
        QGridLayout * gridLayout = new QGridLayout(tab);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

        QVBoxLayout * verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetMinimumSize);

        QHBoxLayout * horizontalLayout_top = new QHBoxLayout();
        horizontalLayout_top->setObjectName(QString::fromUtf8("horizontalLayoutTop"));
        horizontalLayout_top->setSizeConstraint(QLayout::SetNoConstraint);

        {
            rectificationSetting = new QComboBox(tab);
            rectificationSetting->setObjectName(QString::fromUtf8("rectificationSetting"));
            QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Maximum);
            sizePolicy1.setHorizontalStretch(0);
            sizePolicy1.setVerticalStretch(0);
            sizePolicy1.setHeightForWidth(rectificationSetting->sizePolicy().hasHeightForWidth());
            rectificationSetting->setSizePolicy(sizePolicy1);
            horizontalLayout_top->addWidget(rectificationSetting);

            rectificationSetting->addItem("Fisheye");
            rectificationSetting->addItem("Rectified");
            rectificationSetting->addItem("ORB Feature extraction");
        }

        QSizePolicy buttonSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        buttonSizePolicy.setHorizontalStretch(0);
        buttonSizePolicy.setVerticalStretch(0);

        {
            QPushButton * btn = new QPushButton(tab);
            btn->setObjectName(QString::fromUtf8("saveLeft"));
            btn->setText("Save Left Image");
            buttonSizePolicy.setHeightForWidth(btn->sizePolicy().hasHeightForWidth());
            btn->setSizePolicy(buttonSizePolicy);
            horizontalLayout_top->addWidget(btn);
            connect(btn, &QPushButton::clicked, [=]() { SaveImage(0); });
        }

        {
            QPushButton * btn = new QPushButton(tab);
            btn->setObjectName(QString::fromUtf8("saveRight"));
            btn->setText("Save Right Image");
            btn->setSizePolicy(buttonSizePolicy);
            horizontalLayout_top->addWidget(btn);
            connect(btn, &QPushButton::clicked, [=]() { SaveImage(1); });
        }

        QHBoxLayout * horizontalLayout_bottom = new QHBoxLayout();
        horizontalLayout_bottom->setObjectName(QString::fromUtf8("horizontalLayoutBottom"));
        horizontalLayout_bottom->setSizeConstraint(QLayout::SetMinimumSize);

        horizontalLayout_bottom->addWidget(&leftImage);
        horizontalLayout_bottom->addWidget(&rightImage);

        verticalLayout->addLayout(horizontalLayout_top);
        verticalLayout->addLayout(horizontalLayout_bottom);
        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);
        tabWidget->addTab(tab, "Image");
    }

    ui->visualizerLeft->AddFrame(Eigen::Affine3f::Identity(), "Origin", 0.4);
    ui->visualizerLeft->AddFrame(Eigen::Affine3f::Identity(), "Pose", 0.3);
    //ui->visualizerLeft->AddFrame(Eigen::Affine3f::Identity(), "Left", 0.2);
    //ui->visualizerLeft->AddFrame(Eigen::Affine3f::Identity(), "Right", 0.2);
    ui->visualizerRight->AddFrame(Eigen::Affine3f::Identity(), "Origin", 0.4);
    ui->visualizerRight->AddFrame(Eigen::Affine3f::Identity(), "Pose", 0.3);

    ResetView();

    realsense->RegisterCallback_Pose(std::bind(&RealsenseTab::PoseUpdate, this, std::placeholders::_1));
    realsense->RegisterCallback_LeftImage(std::bind(&RealsenseTab::ImageUpdate, this, 0, std::placeholders::_1), false);
    realsense->RegisterCallback_RightImage(std::bind(&RealsenseTab::ImageUpdate, this, 1, std::placeholders::_1), false);

    orb = cv::ORB::create(32, 1.0f, 1, 31, 0);
    sift = cv::xfeatures2d::SIFT::create(32);
}

RealsenseTab::~RealsenseTab()
{
    realsense->ClearCallbacks();
    realsense->Disconnect();
    delete ui;
}

void RealsenseTab::on_resetViewBtn_clicked()
{
    ResetView();
}

void RealsenseTab::ResetView()
{
    ui->visualizerLeft->SetViewpoint(Eigen::Vector3f(-38.7, 73, 50.8), Eigen::Vector3f(0, 0, -3.6));
    ui->visualizerRight->SetViewpoint(Eigen::Vector3f(0, 0, 90), Eigen::Vector3f(0, 0, -6));
}

void RealsenseTab::PoseUpdate(Eigen::Affine3f pose)
{
    ui->visualizerLeft->UpdateFrame("Pose", pose);
    ui->visualizerRight->UpdateFrame("Pose", pose);
    //ui->visualizerLeft->UpdateFrame("Left", W_T_LEFT);
    //ui->visualizerLeft->UpdateFrame("Right", W_T_RIGHT);

    auto XYZ = Realsense::GetPoseTranslation(pose);
    auto RPY = Realsense::GetPoseEulerRPY(pose, true);

    QString text;
    text.sprintf("RPY: %+.2f, %+.2f, %+.2f", RPY(0), RPY(1), RPY(2));
    ui->angles->setText(text);

    text.sprintf("XYZ: %+.2f, %+.2f, %+.2f", XYZ(0), XYZ(1), XYZ(2));
    ui->position->setText(text);
}

Eigen::Vector3f RealsenseTab::GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm)
{
    Eigen::Vector3f eul; // yaw, pitch, roll

    // convention used by (*) and (**).
    // note: the final orientation is the same as in XYZ order about fixed axes ...
    if (rotm(2,0) < 1) {
        if (rotm(2,0) > -1) { // case 1: if r31 ~= ±1
            // Solution with positive sign. It limits the range of the values
            // of theta_y to (-pi/2, pi/2):
            eul(0) = atan2f(rotm(1,0), rotm(0,0)); // theta_z
            eul(1) = asinf(-rotm(2,0));            // theta_y
            eul(2) = atan2f(rotm(2,1), rotm(2,2)); // theta_x
        } else { // case 2: if r31 = -1
            // theta_x and theta_z are linked --> Gimbal lock:
            // There are infinity number of solutions for theta_x - theta_z = atan2(-r23, r22).
            // To find a solution, set theta_x = 0 by convention.
            eul(0) = -atan2f(-rotm(1,2), rotm(1,1));
            eul(1) = M_PI / 2.f;
            eul(2) = 0;
        }
    } else { // case 3: if r31 = 1
        // Gimbal lock: There is not a unique solution for
        //   theta_x + theta_z = atan2(-r23, r22), by convention, set theta_x = 0.
        eul(0) = atan2(-rotm(1,2), rotm(1,1));
        eul(1) = -M_PI / 2.f;
        eul(2) = 0;
    }

    return eul;
}

Eigen::Vector3f RealsenseTab::GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform)
{
    return GetEulerAnglesZYX(static_cast<Eigen::Matrix<float, 3, 3>>(transform.block<3,3>(0,0).matrix()));
}

void RealsenseTab::on_resetOriginBtn_clicked()
{
    realsense->ResetOrigin();
}

void RealsenseTab::on_getViewBtn_clicked()
{
    auto view = ui->visualizerLeft->GetViewpoint();
    std::cout << view << std::endl;

    //auto euler = Eigen::Affine3f(view).rotation().eulerAngles(2,1,0); // this does not seem to work
    auto euler = GetEulerAnglesZYX(view);
    std::cout << "Roll,Pitch,Yaw: " << rad2deg(euler(2)) << ", " << rad2deg(euler(1)) << ", " << rad2deg(euler(0)) << std::endl;
}

void RealsenseTab::on_getOrientationBtn_clicked()
{
    auto pose = realsense->GetPose();
    std::cout << pose.matrix() << std::endl;

    //auto euler = Eigen::Affine3f(W_T_Pp).rotation().eulerAngles(2,1,0); // this does not seem to work
    auto euler = GetEulerAnglesZYX(pose.matrix());
    std::cout << "Roll,Pitch,Yaw: " << rad2deg(euler(2)) << ", " << rad2deg(euler(1)) << ", " << rad2deg(euler(0)) << std::endl;
}

void RealsenseTab::ImageUpdate(uint8_t side, cv::Mat image)
{
    if (rectificationSetting->currentIndex() == 0) { // Raw/fisheye image
        if (side == 0)
            leftImage.update(image);
        else if (side == 1)
            rightImage.update(image);
    }
    else if (rectificationSetting->currentIndex() == 1) { // Rectified image
        if (side == 0) {
            cv::Mat img = realsense->GetLeftImage(true);
            leftImage.update(img);
        } else if (side == 1) {
            cv::Mat img = realsense->GetRightImage(true);
            rightImage.update(img);
        }
    }
    else if (rectificationSetting->currentIndex() == 2 && side == 0) { // ORB Feature extraction
        cv::Mat left = realsense->GetLeftImage(false);
        cv::Mat right = realsense->GetRightImage(false);

        cv::Mat out_left;
        cv::Mat out_right;

        std::vector<cv::KeyPoint> keypoints_left;
        cv::Mat descriptors_left;
        cv::Mat mask_left;

        orb->detect(left, keypoints_left);
        orb->compute(left, keypoints_left, descriptors_left);

        cv::drawKeypoints(left, keypoints_left, out_left);

        std::vector<cv::KeyPoint> keypoints_right;
        cv::Mat descriptors_right;
        cv::Mat mask_right;

        orb->detect(right, keypoints_right);
        orb->compute(right, keypoints_right, descriptors_right);

        cv::drawKeypoints(right, keypoints_right, out_right);

        cv::Mat combined;
        //cv::hconcat(left_out, right_out, combined);

        bool knnMatcherEnabled = false;
        if (!knnMatcherEnabled) {
            cv::BFMatcher matcher(cv::NORM_L2, true);
            std::vector<cv::DMatch> matches;
            matcher.match(descriptors_left, descriptors_right, matches);
            cv::drawMatches(left, keypoints_left, right, keypoints_right, matches, combined);
        } else {
            cv::BFMatcher matcher(cv::NORM_L2, false);
            std::vector<std::vector<cv::DMatch>> matches;
            matcher.knnMatch(descriptors_left, descriptors_right, matches, 2);

            std::vector<cv::DMatch> matches1;
            std::vector<cv::DMatch> matches2;
            for(int i=0; i < matches.size(); i++)
            {
                matches1.push_back(matches[i][0]);
                matches2.push_back(matches[i][1]);
            }

            cv::Mat img_matches1, img_matches2;
            cv::drawMatches(left, keypoints_left, right, keypoints_right, matches1, img_matches1);
            cv::drawMatches(left, keypoints_left, right, keypoints_right, matches2, img_matches2);

            cv::vconcat(img_matches1, img_matches2, combined);
        }

        leftImage.update(combined);

        cv::Mat empty;
        rightImage.update(empty);
    }
}

void RealsenseTab::SaveImage(bool side)
{
    cv::Mat img;
    bool rectified = (rectificationSetting->currentIndex() == 1);

    if (side == 0)
        img = realsense->GetLeftImage(rectified);
    else
        img = realsense->GetRightImage(rectified);

    QString FilePath = QFileDialog::getSaveFileName(this, "Save image", ".", "Image (*.png)");

    if (FilePath.isEmpty()) {
        QMessageBox::critical( this, "Save image", QString("File not specified!"), QMessageBox::Ok );
        return;
    }

    if (side == 0)
        leftImage.update(img);
    else
        rightImage.update(img);

    auto info = QFileInfo(FilePath);
    qDebug() << info.filePath();
    qDebug() << info.fileName();
    qDebug() << info.baseName();
    qDebug() << info.completeSuffix();

    if (!FilePath.isEmpty()) {
        QString extension = QFileInfo(FilePath).completeSuffix();
        if (!extension.compare("png")) {
            cv::imwrite(FilePath.toStdString(), img);
        }
        else {
            cv::imwrite(FilePath.toStdString() + ".png", img);
        }
    }
}
