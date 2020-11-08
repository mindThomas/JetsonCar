#ifndef TESTVIEW_H
#define TESTVIEW_H

#include <QMainWindow>

namespace Ui {
class TestView;
}

class TestView : public QMainWindow
{
    Q_OBJECT

public:
    explicit TestView(QWidget *parent = nullptr);
    ~TestView();

private slots:
    void ButtonPress(bool pressed);

private:
    Ui::TestView *ui;

    int count;
};

#endif // TESTVIEW_H
