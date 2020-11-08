#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <qmessagebox.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("Test GUI"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    EASY_BLOCK("Button pressed");
    ui->label->setText(QString::fromUtf8("Button was pressed"));
    QMessageBox::information( this, "Test GUI", "You pressed the button!", QMessageBox::Ok );
    ui->label->setText(QString::fromUtf8("Press the button"));
    EASY_END_BLOCK;
}
