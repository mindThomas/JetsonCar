#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <easy/profiler.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_actionOpen_PCD_triggered();

private:
    void checkForReady(void);

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
