#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void updateChart();
    unsigned int* ReadADC();
    void DropBoxCallback(int index);
    void ChkBxCallback(int index);
};

#endif // MAINWINDOW_H
