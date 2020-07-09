#ifndef FILE_CAM_GCODE
#define FILE_CAM_GCODE
/*
    This header file is part of CadCam.

    This header file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This header file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CadCam.  If not, see <https://www.gnu.org/licenses/>.

    AUTOR     : Grotius
    DATE      : June 2020
    REVISION  : 7.6.2020     Created class
                9.6.2020     Added area & contour swap
                10.6.2020    Added Gcode output for linestrips, ellipse, bezier curve, polygon, rectangular
    TESTED    : 9-6-2020     Tested Linuxcnc quadratic and cubic splines and nurbes, the linuxcnc splines are useless..
    TODO      : Expand Linuxcnc source code with gcode for : ellipse, polynomial splines (a wannahave for robot trajectory), and bezier
                All these are in line with CadCam, Freecad, Inkscape and Librecad dxf output.
    USAGE     : To use this header only library/file, include also structs.h to your project.
    PROTOCOL  : 1. Minimize the use of QT libraries to stay compatible with other program's.
    NOTES     : Control your gcode output with : https://ncviewer.com/

*/

#include <iostream>
#include <algorithm>
#include <structs.h>
#include <math.h>
#include <chrono> //high_resolution_clock
#include <thread>
#include <fstream>
#include <unistd.h> //user for find current directory
#include <variable.h>

std::chrono::duration<double> elapsed;

extern double feedrate;
extern double cutheight;
extern double rapidheight;
extern double pierceheight;
extern double piercespeed;
extern double piercedelay;
extern bool spline_G5;
extern std::string gcode_filename;

void area();
double sum_plus_controlpoints(int i);
void map_cw_ccw();
void swap_contour(std::vector<CONTOUR>::iterator it);
void process_gcode();

void gcode(int i);
void write(std::string text);
void intro();
void outtro();
void M3(int );
void M5();
std::vector<POINT> spline_as_linestrip(std::vector<std::vector<double>> C);
std::vector<POINT> ellipse_as_linestrip(int i);
std::vector<POINT> bezier_as_linestrip(int i);
std::string get_current_dir();

void process_gcode(){
    auto start = std::chrono::high_resolution_clock::now();

    std::ofstream myfile;
    myfile.open(get_current_dir()+"/"+gcode_filename /*, std::ios::app*/); //ios app = append true
    myfile.clear();

    intro();
    //print contour sequence, based on keep parts together algoritme
    for(int i=0; i<sequence.size(); i++){
        //std::cout<<"contour: "<<sequence.at(i)<<std::endl;

        for(int j=0; j<contours.at(sequence.at(i)).sequence.size(); j++){
            int object=contours.at(sequence.at(i)).sequence.at(j);

            if(object==contours.at(sequence.at(i)).sequence.front()){
                M3(object);
            }

            gcode(object);

            if(object==contours.at(sequence.at(i)).sequence.size()){
                M5();
            }
        }
    }
    M5();
    outtro();

    myfile.close();

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    //    std::cout<<""<<std::endl;
    //    std::cout<<"time: "<<elapsed.count()<<" sec"<<std::endl;
    //    std::cout<<"Gcode done!"<<std::endl;
}

void gcode(int i){

    //line section
    if(cad.at(i).type=="line"){
        write("G1 X"+std::to_string(cad.at(i).end.x)+ " Y"+std::to_string(cad.at(i).end.y)+ " Z"+std::to_string(cutheight));
    }

    if(cad.at(i).type=="linestrip" || cad.at(i).type=="polygon" || cad.at(i).type=="rectangular"){
        for(int j=0; j<cad.at(i).control.size(); j++){
            write("G1 X"+std::to_string(cad.at(i).control.at(j).x)+ " Y"+std::to_string(cad.at(i).control.at(j).y)+ " Z"+std::to_string(cutheight));
        }
        write("G1 X"+std::to_string(cad.at(i).end.x)+ " Y"+std::to_string(cad.at(i).end.y)+ " Z"+std::to_string(cutheight));
    }

    if(cad.at(i).type=="ellipse"){
        std::vector<POINT>::iterator it;
        std::vector<POINT>points=ellipse_as_linestrip(i);
        for(it=points.begin()+1; it<points.end(); it++){ //exclude the ellipse startpoint, begin+1
            write("G1 X"+std::to_string(it->x)+ " Y"+std::to_string(it->y));
        }
        write("G1 X"+std::to_string(cad.at(i).end.x)+ " Y"+std::to_string(cad.at(i).end.y));
    }

    if(cad.at(i).type=="bezier"){
        std::vector<POINT>::iterator it;
        std::vector<POINT>points=bezier_as_linestrip(i);
        for(it=points.begin()+1; it<points.end(); it++){ //exclude the bezier startpoint, begin+1
            write("G1 X"+std::to_string(it->x)+ " Y"+std::to_string(it->y));
        }
        write("G1 X"+std::to_string(cad.at(i).end.x)+ " Y"+std::to_string(cad.at(i).end.y));
    }

    //circle section
    //I=offset xcenter-xstart, J=offset ycenter-ystart, G2=clockwise (cw), G3=counterclockwise (ccw)
    if(cad.at(i).type=="circle"){ //circle is single contour, is always at vector front position
        if(cad.at(i).red==255 && cad.at(i).green==0 && cad.at(i).blue==0 && cad.at(i).alpha==255){
            write("G2 I"+std::to_string(cad.at(i).center.x-cad.at(i).start.x)+" J"+std::to_string(cad.at(i).center.y-cad.at(i).start.y));
        } else { //color must be yellow, ccw
            write("G3 I"+std::to_string(cad.at(i).center.x-cad.at(i).start.x)+" J"+std::to_string(cad.at(i).center.y-cad.at(i).start.y));
        }
    }

    //arc section
    //X=xend, Y=yend. For arc given a G0 startposition and a XY endposition. http://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g2-g3
    //I=offset xcenter-xstart, J=offset ycenter-ystart, G2=clockwise (cw), G3=counterclockwise (ccw)
    if(cad.at(i).type=="arc"){
        if(cad.at(i).d>0){
            write("G2 X"+std::to_string(cad.at(i).end.x)+" Y"+std::to_string(cad.at(i).end.y)+" I"+std::to_string(cad.at(i).center.x-cad.at(i).start.x)+" J"+std::to_string(cad.at(i).center.y-cad.at(i).start.y));
        }
        if(cad.at(i).d<0){
            write("G3 X"+std::to_string(cad.at(i).end.x)+" Y"+std::to_string(cad.at(i).end.y)+" I"+std::to_string(cad.at(i).center.x-cad.at(i).start.x)+" J"+std::to_string(cad.at(i).center.y-cad.at(i).start.y));
        }
    }

    if(cad.at(i).type=="spline" && spline_G5==false){ //write spline gcode as a linestrip
        std::vector<std::vector<double>> C; //2d array
        std::vector<POINT> points;
        std::vector<POINT>::iterator it;
        C.push_back({cad.at(i).start.x, cad.at(i).start.y}); //spline startpoint
        for(int j=0; j<cad.at(i).control.size(); j++){
            C.push_back({cad.at(i).control.at(j).x, cad.at(i).control.at(j).y}); //spline controlpoints
        }
        C.push_back({cad.at(i).end.x, cad.at(i).end.y}); //spline endpoint
        points=spline_as_linestrip(C);
        for(it=points.begin()+1; it<points.end(); it++){ //exclude the spline startpoint, begin+1
            write("G1 X"+std::to_string(it->x)+ " Y"+std::to_string(it->y));
        }
        //added....
        write("G1 X"+std::to_string(cad.at(i).end.x)+ " Y"+std::to_string(cad.at(i).end.y));
    }

    //spline section http://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g5
}

void intro(){
    write("G21      (unit:mm)");
    write("G40      (cutter compensation off)");
    write("G90      (absolute distance, no offsets)");
    write("G64P0.01 (path following accuracy)");
    write("F1       (initialize feedrate)");
    write("G0 Z"+std::to_string(rapidheight));
}

void outtro(){
    write("M30 (program end)");
    write("(with great power comes great responsibility)");
}

void M3(int i){
    write("G0 X"+std::to_string(cad.at(i).start.x)+ " Y"+std::to_string(cad.at(i).start.y)+ " Z"+std::to_string(rapidheight));
    write("G0 Z"+std::to_string(pierceheight)+" (pierceheight)");
    write("M3 (torch on)");
    write("G4 P"+std::to_string(piercedelay)+" (pierce delay)");
    write("G1 Z"+std::to_string(cutheight)+" F"+std::to_string(piercespeed)+" (cutheight)");
    write("F"+std::to_string(feedrate));
}

void M5(){
    write("M5 (torch off)");
    write("G0 Z"+std::to_string(rapidheight));
}

void write(std::string text){
    //std::cout<<text<<std::endl;
    std::ofstream myfile;
    myfile.open(get_current_dir()+"/"+gcode_filename, std::ios::app);
    myfile << text << std::endl;
}

void area(){
    std::vector<CONTOUR>::iterator it;
    std::vector<int>::iterator it1;
    for(it=contours.begin(); it<contours.end(); it++){
        double sum=0;
        if(it->contourtype!=0){  //0=open, 1=closed, 2=single object closed
            for(it1=it->sequence.begin();  it1<it->sequence.end(); it1++){
                sum+=sum_plus_controlpoints(*it1);
            }
        }
        it->area=sum/2;
        //std::cout<<"area: "<<sum/2<<std::endl;
    }
}

double sum_plus_controlpoints(int i){
    double sum=0;
    //if object has controlpoints
    if(cad.at(i).control.size()>0){
        sum+= (cad.at(i).control.front().x-cad.at(i).start.x) * (cad.at(i).control.front().y+cad.at(i).start.y);
        for(int j=0; j<cad.at(i).control.size()-1; j++){
            sum+= (cad.at(i).control.at(j+1).x-cad.at(i).control.at(j).x) * (cad.at(i).control.at(j+1).y+cad.at(i).control.at(j).y);
        }
        sum+= (cad.at(i).end.x-cad.at(i).control.back().x) * (cad.at(i).end.y+cad.at(i).control.back().y);
    }

    //if object has no controlpoints
    if(cad.at(i).control.size()==0){
        sum+= (cad.at(i).end.x-cad.at(i).start.x) * (cad.at(i).end.y+cad.at(i).start.y);
    }
    return sum;
}

void map_cw_ccw(){
    std::vector<CONTOUR>::iterator it;
    for(it=contours.begin(); it<contours.end(); it++){
        if(it->area==0){
            //do nothing, open contour
        }
        if(it->area>0){
            //cw contour, check if contour color is red, if not, swap contour
            int object=it->sequence.front();

            if(cad.at(object).red==255 && cad.at(object).green==0 && cad.at(object).blue==0 && cad.at(object).alpha==255){
                //std::cout<<"cw contour, color is red, ok"<<std::endl;
            }
            if(cad.at(object).red==255 && cad.at(object).green==255 && cad.at(object).blue==0 && cad.at(object).alpha==255){
                //std::cout<<"cw contour, color is yellow, not ok, swap"<<std::endl;
                swap_contour(it);
                it->area=it->area*-1; //keep the area value in line with the color
            }
        }
        if(it->area<0){
            int object=it->sequence.front();

            if(cad.at(object).red==255 && cad.at(object).green==0 && cad.at(object).blue==0 && cad.at(object).alpha==255){
                //std::cout<<"ccw contour, color is red, not ok, swap"<<std::endl;
                swap_contour(it);
                it->area=it->area*-1;
            }
            if(cad.at(object).red==255 && cad.at(object).green==255 && cad.at(object).blue==0 && cad.at(object).alpha==255){
                //std::cout<<"ccw contour, color is yellow, ok"<<std::endl;
            }
        }
    }
}
void swap_contour(std::vector<CONTOUR>::iterator it){
    std::vector<int>::iterator it1;
    std::reverse(it->sequence.begin(), it->sequence.end());
    for(it1=it->sequence.begin(); it1<it->sequence.end(); it1++){
        //swap it1 parameters
        double start_x = cad.at(*it1).start.x;
        double start_y = cad.at(*it1).start.y;
        double start_z = cad.at(*it1).start.z;
        cad.at(*it1).start.x=cad.at(*it1).end.x;
        cad.at(*it1).start.y=cad.at(*it1).end.y;
        cad.at(*it1).start.z=cad.at(*it1).end.z;
        cad.at(*it1).end.x=start_x;
        cad.at(*it1).end.y=start_y;
        cad.at(*it1).end.z=start_z;
        std::reverse(cad.at(*it1).control.begin(), cad.at(*it1).control.end());
    }
}

std::vector<POINT> spline_as_linestrip(std::vector<std::vector<double>> C){

    POINT point;
    std::vector<POINT> points;

    double S[C.size()-1][2][4];
    double w, b[C.size()], d[C.size()], x[C.size()];
    int i, dim;
    int n = C.size()-1; // number of splines

    for(dim=0; dim<2; dim++)
    {
        // define d[]:
        d[0]=3.0*(C[1][dim]-C[0][dim]);
        for(i=1; i<n; i++)
            d[i]=3.0*(C[i+1][dim]-C[i-1][dim]);
        d[n]=3.0*(C[n][dim]-C[n-1][dim]);
        // forward sweep: (simplified tridiagonal solution: all a[i]=1 and all c[i]=1)
        b[0]=2.0;
        for(i=1; i<C.size(); i++)
        {
            w = 1.0/b[i-1];
            b[i] = 4.0 - w;
            d[i] -= (w*d[i-1]);
        }
        // calculate solution vector x[i] = D[i]:
        // (Wikipedia x[] = Wolfram D[])
        x[n]=d[n]/b[n];
        for(i=n-1; i>=0; i--)
            x[i]=(d[i]-x[i+1])/b[i];
        // calculate spline S[i][dim] a, b, c and d:
        for(i=0; i<n; i++)
        {
            S[i][dim][0]=C[i][dim];
            S[i][dim][1]=x[i];
            S[i][dim][2]=3.0*(C[i+1][dim]-C[i][dim]) - 2.0*x[i] - x[i+1];
            S[i][dim][3]=2.0*(C[i][dim]-C[i+1][dim]) + x[i] + x[i+1];
        }
    }

    for(int p=0; p<C.size()-1; p++)  //spline points
    {
        for(double t=0; t<1; t+=resolution)  //time 0-1 for each spline
        {
            point.x = ((S[p][0][3]*t+S[p][0][2])*t+S[p][0][1])*t+S[p][0][0];
            point.y = ((S[p][1][3]*t+S[p][1][2])*t+S[p][1][1])*t+S[p][1][0];
            points.push_back(point);
        }
    }
    return points;
}

std::vector<POINT> ellipse_as_linestrip(int i){

    POINT point;
    std::vector<POINT> points;

    //ellipse center = xcen, ycen
    //ellipse base right = xend, yend
    //ellipse top point = xinput, yinput

    double center_x = cad.at(i).center.x;
    double center_y = cad.at(i).center.y;

    //center point to endpoint mayor axis..
    double radius_x = sqrt(pow(cad.at(i).end.x-cad.at(i).center.x,2) + pow(cad.at(i).end.y-cad.at(i).center.y,2));
    //ratio minor axis to mayor axis..
    double radius_y=0;

    if(cad.at(i).ratio==0){ //takes the controlpoint instead of ratio factor
        radius_y = sqrt(pow(cad.at(i).input.x-cad.at(i).center.x,2) + pow(cad.at(i).input.y-cad.at(i).center.y,2));
    }
    if(cad.at(i).ratio!=0){ //if ratio from dxf is available, it takes its value
        radius_y = cad.at(i).ratio*radius_x;
    }

    //calculate angle of ellipse
    double x1=cad.at(i).end.x;
    double y1=cad.at(i).end.y;
    double radian = atan2(y1 - center_y, x1 - center_x);
    double angle = radian * (180 / M_PI);
    if (angle < 0.0)
        angle += 360.0;

    for(double j = cad.at(i).pi_angle_start; j<cad.at(i).pi_angle_end; j+=resolution) //0.01
    {
        //non rotated ellipse..
        double non_rotated_x = /*center_x +*/ cos(j)*radius_x;
        double non_rotated_y = /*center_y +*/ sin(j)*radius_y;
        //rotated ellipse..
        point.x = center_x  + cos(angle * M_PI / 180.0 )* non_rotated_x - sin(angle * M_PI / 180.0 ) * non_rotated_y;
        point.y = center_y  + sin(angle * M_PI / 180.0 )* non_rotated_x + cos(angle * M_PI / 180.0 ) * non_rotated_y;
        points.push_back(point);
    }
    return points;
}

std::vector<POINT> bezier_as_linestrip(int i){

    POINT point;
    std::vector<POINT> points;
    for(double t=0; t<1; t+=resolution){
        //Calculate Point E, on the line AB
        double Ex = ((1-t) * cad.at(i).start.x) + (t * cad.at(i).control.at(0).x);
        double Ey = ((1-t) * cad.at(i).start.y) + (t * cad.at(i).control.at(0).y);
        double Ez = ((1-t) * cad.at(i).start.z) + (t * cad.at(i).control.at(0).z);

        //Calculate Point F, on the line BC
        double Fx = ((1-t) * cad.at(i).control.at(0).x) + (t * cad.at(i).control.at(1).x);
        double Fy = ((1-t) * cad.at(i).control.at(0).y) + (t * cad.at(i).control.at(1).y);
        double Fz = ((1-t) * cad.at(i).control.at(0).z) + (t * cad.at(i).control.at(1).z);

        //Calculate Point G, on the line CD
        double Gx = ((1-t) * cad.at(i).control.at(1).x) + (t * cad.at(i).end.x);
        double Gy = ((1-t) * cad.at(i).control.at(1).y) + (t * cad.at(i).end.y);
        double Gz = ((1-t) * cad.at(i).control.at(1).z) + (t * cad.at(i).end.z);

        //Calculate Point Q, on the line EF
        double Qx = ((1-t) * Ex) + (t * Fx);
        double Qy = ((1-t) * Ey) + (t * Fy);
        double Qz = ((1-t) * Ez) + (t * Fz);

        //Calculate Point R, on the line FG
        double Rx = ((1-t) * Fx) + (t * Gx);
        double Ry = ((1-t) * Fy) + (t * Gy);
        double Rz = ((1-t) * Fz) + (t * Gz);

        //Calculate Point P, on the line QR
        point.x = ((1-t) * Qx) + (t * Rx);
        point.y = ((1-t) * Qy) + (t * Ry);
        point.z = ((1-t) * Qz) + (t * Rz);
        points.push_back(point);
    }
    return points;
}

std::string get_current_dir() { //use std:: libs instead of qt libs
    char buff[FILENAME_MAX]; //create string buffer to hold path
    getcwd( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}

#endif























