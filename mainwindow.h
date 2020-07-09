#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QStringList>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

protected:

    void timerEvent(QTimerEvent *);

private slots:

    void on_toolButton_line_pressed();

    void on_toolButton_open_pressed();

    void on_toolButton_spline_pressed();

    void on_toolButton_arc_pressed();

    void on_toolButton_circle_pressed();

    void on_toolButton_rectangular_pressed();

    void on_toolButton_linestrip_pressed();

    void on_toolButton_ellipse_pressed();

    void on_toolButton_bezier_pressed();

    void on_toolButton_polygon_pressed();

    void on_toolButton_new_pressed();

    void on_pushButton_process_pressed();

    void on_toolButton_cam_pressed();

    void on_pushButton_close_pressed();

    void on_pushButton_offset_pressed();

    void on_pushButton_clear_pressed();

    void on_pushButton_show_pocket_pressed();

    void on_pushButton_delete_arc_segment_pressed();

private:
    Ui::MainWindow *ui;
    int timerId;
};

#endif // MAINWINDOW_H
