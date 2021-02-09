#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "ros_msg_parser/ros_parser.hpp"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <QFileDialog>
#include <QMessageBox>

#include <sensor_msgs/Image.h>

#include <jetsoncar_utils/jetsoncar_utils.h>
#include <jetsoncar_utils/formatting.hpp>
#include <spdlog/stopwatch.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <QSplitter>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    spdlog::set_level(spdlog::level::debug); // Set global log level to info
    setWindowTitle(QStringLiteral("ROSbag analysis"));

    /* Prepare listview */
    // Create model
    itemListModel = new QStringListModel(this);
    itemListModel->setStringList(QStringList{}); // set empty list
    // Glue model and view together
    ui->listView->setModel(itemListModel);

    // Only resize the right part of the splitter when changing size of the window
    ui->splitter->setStretchFactor(0, 0);
    ui->splitter->setStretchFactor(1, 1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_ROS_bag_triggered()
{
    QString FilePath = QFileDialog::getOpenFileName(this, "Select a ROS-bag", ".", "ROS-bag, .bag (*.bag)");

    if (FilePath.isEmpty()) {
        QMessageBox::critical( this, "Open ROS-bag", QString("File not specified!"), QMessageBox::Ok );
        return;
    }

    if (!utils::FileExist(FilePath.toStdString())) {
        QMessageBox::critical( this, "Open ROS-bag", QString("File does not exist {%1}!").arg(FilePath), QMessageBox::Ok );
        return;
    }

    if (utils::getFileExtension(FilePath.toStdString()).compare(".bag")) {
        QMessageBox::critical( this, "Open ROS-bag", QString("Incorrect file type %1 (should be .bag)").arg(QString::fromStdString(utils::getFileExtension(FilePath.toStdString()))), QMessageBox::Ok );
        return;
    }

    // Reading and parsing of undefined message types that can be cast to Double or String
    // https://github.com/facontidavide/ros_msg_parser
    rosbag::Bag bag;
    RosMsgParser::ParsersCollection parsers;
    try
    {
        bag.open(FilePath.toStdString());
    }
    catch (rosbag::BagException& ex)
    {
        QMessageBox::critical( this, "Open ROS-bag", QString("rosbag::open thrown an exception: %1").arg(QString::fromStdString(ex.what())), QMessageBox::Ok );
        return;
    }

    /* See http://wiki.ros.org/rosbag/Code%20API */

    // this  rosbag::View will accept ALL the messages
    rosbag::View bag_view(bag);

    // register (only once at the beginning) the type of messages
    QStringList list;
    spdlog::debug("Bag topics:");
    for (const rosbag::ConnectionInfo* connection : bag_view.getConnections())
    {
        const std::string& topic_name = connection->topic;
        parsers.registerParser(topic_name, *connection);
        spdlog::debug("{}", topic_name);
        list.push_back(QString::fromStdString(topic_name));
    }

    itemListModel->setStringList(list);

    /*for (rosbag::MessageInstance msg_instance : bag_view)
    {
        const std::string& topic_name = msg_instance.getTopic();
        const auto deserialized_msg = parsers.deserialize(topic_name, msg_instance);

        // Print the content of the message
        spdlog::debug("--------- {} ----------", topic_name.c_str());
        for (const auto& it : deserialized_msg->renamed_vals)
        {
            const std::string& key = it.first;
            const double value = it.second;
            spdlog::debug("{} = {}", key, value);
        }
        for (const auto& it : deserialized_msg->flat_msg.name)
        {
            const std::string& key = it.first.toStdString();
            const std::string& value = it.second;
            spdlog::debug("{} = {}", key, value);
        }
    }*/

    /*for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
      if (i != nullptr)
        std::cout << i->data << std::endl;
    }*/    
}

void ROSbag_Load_test()
{
     QString FilePath = "";

    {
        // Regular reading and parsing with known message type
        // http://wiki.ros.org/rosbag/Code%20API
        rosbag::Bag bag;
        bag.open(FilePath.toStdString());  // BagMode is Read by default

        for(rosbag::MessageInstance const m: rosbag::View(bag))
        {
            sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
            if (img != nullptr) {
                spdlog::debug("Parsed image from topic: {} [{} x {}]", m.getTopic(), img->width, img->height);
            }
        }

        bag.close();
    }

    {
        // Reading and parsing of undefined message types that can be cast to Double or String
        // https://github.com/facontidavide/ros_msg_parser
        rosbag::Bag bag;
        RosMsgParser::ParsersCollection parsers;
        try
        {
            bag.open(FilePath.toStdString());
        }
        catch (rosbag::BagException& ex)
        {
            spdlog::error("rosbag::open thrown an exception: {}", ex.what());
            return;
        }

        // this  rosbag::View will accept ALL the messages
        rosbag::View bag_view(bag);

        // register (only once at the beginning) the type of messages
        spdlog::debug("Bag topics:");
        for (const rosbag::ConnectionInfo* connection : bag_view.getConnections())
        {
            const std::string& topic_name = connection->topic;
            parsers.registerParser(topic_name, *connection);
            spdlog::debug("{}", topic_name);
        }

        for (rosbag::MessageInstance msg_instance : bag_view)
        {
            const std::string& topic_name = msg_instance.getTopic();
            const auto deserialized_msg = parsers.deserialize(topic_name, msg_instance);

            // Print the content of the message
            spdlog::debug("--------- {} ----------", topic_name.c_str());
            for (const auto& it : deserialized_msg->renamed_vals)
            {
                const std::string& key = it.first;
                const double value = it.second;
                spdlog::debug("{} = {}", key, value);
            }
            for (const auto& it : deserialized_msg->flat_msg.name)
            {
                const std::string& key = it.first.toStdString();
                const std::string& value = it.second;
                spdlog::debug("{} = {}", key, value);
            }
        }
     }
}
