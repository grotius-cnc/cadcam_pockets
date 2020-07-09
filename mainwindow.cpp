#include <mainwindow.h>
#include "ui_mainwindow.h"
#include <opengl.h>
#include <variable.h>
#include <structs.h>
#include <iostream>
#include <dxf/dxf_read_line.h>
#include <dxf/dxf_read_spline.h>
#include <dxf/dxf_read_circle.h>
#include <dxf/dxf_read_arc.h>
#include <dxf/dxf_read_ellipse.h>
#include <dxf/dxf_read_dimension.h>
#include <dxf/dxf_read_text.h>
#include <cam/cam_contours.h>
#include <cam/cam_dept_sequence.h>
#include <cam/cam_gcode.h>
#include <cam/cam_offset.h>
#include <lib/lib_offset.h>
#include <lib/lib_pocket.h>

double feedrate;
double cutheight;
double rapidheight;
double pierceheight;
double piercespeed;
double piercedelay;
bool spline_G5;
std::string gcode_filename;
int pocketnr;

#define MyQLineEdit() << QLineEdit() << fixed << qSetRealNumberPrecision(3)

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timerId = startTimer(200);

    ui->frame_side->hide();
    //example how to connect to buttons:
    //connect(ui->toolButton_new, SIGNAL(clicked()),this, SLOT(on_toolButton_arc_pressed()));
}

MainWindow::~MainWindow()
{
    killTimer(timerId);
    delete ui;
}

void MainWindow::timerEvent(QTimerEvent *)
{
    ui->lineEdit_mx->setText(QString::number(mx,'f',6));
    ui->lineEdit_my->setText(QString::number(my,'f',6));
    ui->lineEdit_message->setText(QString::fromStdString(message));
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    int_key=event->key();
    //std::cout<<"key:"<<int_key<<std::endl;
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    int_key=-1;
}

void MainWindow::on_toolButton_open_pressed()
{
#define DXF_FILES "AC1027 Inkscape dxf R14 files, AC1021 Librecad dxf 2007 files(*.dxf)"
#define GRO_FILES "gro files (*.gro)"

    QFileDialog *dialog = new QFileDialog;
    dialog->resize(500,250);
    QString selectedFilter;

    QString filename_open = QFileDialog::getOpenFileName(
                dialog,
                "Open",
                QDir::currentPath(),
                DXF_FILES ";;" GRO_FILES,
                &selectedFilter,
                QFileDialog::DontUseNativeDialog); //this solves a dxf multiload problem

    if(selectedFilter == GRO_FILES){
        //(read_gro(filename_open));
    }
    if(selectedFilter == DXF_FILES){
        read_line(std::string(filename_open.toStdString()));
        read_spline(std::string(filename_open.toStdString()));
        read_circle(std::string(filename_open.toStdString()));
        read_arc(std::string(filename_open.toStdString()));
        read_ellipse(std::string(filename_open.toStdString()));
        //read_dimension(std::string(filename_open.toStdString()));
        //read_text(std::string(filename_open.toStdString()));
    }
}

void MainWindow::on_toolButton_line_pressed()
{
    mode="line";
}

void MainWindow::on_toolButton_linestrip_pressed()
{
    mode="linestrip";
}

void MainWindow::on_toolButton_rectangular_pressed()
{
    mode="rectangular";
}

void MainWindow::on_toolButton_circle_pressed()
{
    mode="circle";
}

void MainWindow::on_toolButton_ellipse_pressed()
{
    mode="ellipse";
}

void MainWindow::on_toolButton_arc_pressed()
{
    mode="arc";
}

void MainWindow::on_toolButton_spline_pressed()
{
    mode="spline";
}

void MainWindow::on_toolButton_bezier_pressed()
{
    mode="bezier";
}

void MainWindow::on_toolButton_polygon_pressed()
{
    mode="polygon";
}

void MainWindow::on_toolButton_new_pressed()
{
    cad.clear();
}

void MainWindow::on_toolButton_cam_pressed()
{
    mode="";
    ui->frame_side->show();
    ui->progressBar->setValue(0);
}

void MainWindow::on_pushButton_process_pressed()
{
    mode="";
    feedrate=ui->lineEdit_cutspeed->text().toDouble();
    cutheight=ui->lineEdit_cutheight->text().toDouble();
    rapidheight=ui->lineEdit_travelheight->text().toDouble();
    pierceheight=ui->lineEdit_pierceheight->text().toDouble();
    piercespeed=ui->lineEdit_piercespeed->text().toDouble();
    piercedelay=ui->lineEdit_piercedelay->text().toDouble();
    spline_G5=false;
    gcode_filename=ui->lineEdit_filename->text().toStdString();

    ui->progressBar->setValue(10);
    ui->textEdit_gode->clear();

    find_contours();
    ui->progressBar->setValue(50);
    keep_parts_together();
    ui->progressBar->setValue(60);
    area();
    ui->progressBar->setValue(80);
    map_cw_ccw();
    ui->progressBar->setValue(90);

    //add g2 or g3 to arcs
    for(int i=0; i<cad.size(); i++){
        if(cad.at(i).type=="arc"){
            double xp = cad.at(i).end.x;
            double yp = cad.at(i).end.y;
            double x1 = cad.at(i).start.x;
            double y1 = cad.at(i).start.y;
            double x2 = cad.at(i).control.at(0).x;
            double y2 = cad.at(i).control.at(0).y;
            double d = ((xp-x1)*(y2-y1))-((yp-y1)*(x2-x1));
            if(int(cad.at(i).red)==255 && int(cad.at(i).green)==255){
                if(d>0){//G2
                    cad.at(i).d=d;
                } else
                    if(d<0){//G3
                        cad.at(i).d=d;
                    }
            } else
                if(int(cad.at(i).red)==255 && int(cad.at(i).green)==0){
                    if(d>0){//G2
                        cad.at(i).d=d;
                    } else
                        if(d<0){//G3
                            cad.at(i).d=d;
                        }
                } else { cad.at(i).d=0;}
        }
    }


    process_gcode();
    ui->progressBar->setValue(100);

    //display gcode in textbox
    QString string = QString::fromStdString(gcode_filename);
    QFile file(string);
    QString line;
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)){
        QTextStream stream(&file);
        while (!stream.atEnd()){

            line.append(stream.readLine()+"\n");
        }
        //testLabel->setText(line);
        ui->textEdit_gode->setText(line);
    }
    file.close();


}

void MainWindow::on_pushButton_close_pressed()
{
    ui->frame_side->close();
    ui->progressBar->setValue(0);
}

void MainWindow::on_pushButton_offset_pressed()
{
    offset=ui->doubleSpinBox_offset->value();
    if(offset==0){offset=0.001;}
    cam_offset();

//    std::vector<OBJECT>::iterator it;

//    ui->plainTextEdit_textview->appendPlainText(
//                "G21      (unit:mm)\n"
//                "G40      (cutter compensation off)\n"
//                "G40      (cutter compensation off)\n"
//                "G90      (absolute distance, no offsets)\n"
//                "G64P0.01 (path following accuracy)\n"
//                "F1       (initialize feedrate)\n");

//    for(it=cam.begin(); it<cam.end(); it++){

//        if(it->type=="line"){
//            ui->plainTextEdit_textview->appendPlainText("(start line)");
//            ui->plainTextEdit_textview->appendPlainText("G1 X"+QString::number(it->start.x,'f',3)+" Y"+QString::number(it->start.y,'f',3));
//            ui->plainTextEdit_textview->appendPlainText("G1 X"+QString::number(it->end.x,'f',3)+" Y"+QString::number(it->end.y,'f',3));
//            ui->plainTextEdit_textview->appendPlainText("(end line)");
//        }

//        if(it->type=="arc"){
//            ui->plainTextEdit_textview->appendPlainText("(start arc)");
//            ui->plainTextEdit_textview->appendPlainText("G1 X"+QString::number(it->start.x,'f',3)+" Y"+QString::number(it->start.y,'f',3));
//            ui->plainTextEdit_textview->appendPlainText("G1 X"+QString::number(it->end.x,'f',3)+" Y"+QString::number(it->end.y,'f',3));
//            ui->plainTextEdit_textview->appendPlainText("(end arc)");
//        }
//    }
}

void MainWindow::on_pushButton_clear_pressed()
{
    cam.clear();
}

void MainWindow::on_pushButton_show_pocket_pressed()
{
    std::vector<OBJECT> cleanup=cam;
    cleanup=clean_up_the_pocket(cam);
    cam.clear();
    cam=cleanup;
    cleanup.clear();
}

void MainWindow::on_pushButton_delete_arc_segment_pressed()
{
    cam.erase(cam.begin());
}
