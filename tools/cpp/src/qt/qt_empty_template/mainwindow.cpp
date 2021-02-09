#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <qmessagebox.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("MainWindow"));
}

MainWindow::~MainWindow()
{
    delete ui;
}