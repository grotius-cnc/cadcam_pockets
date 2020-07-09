#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include <string>
#include <math.h>

struct POINT{
    double x,y,z;
};

struct OBJECT {
    //cad section

    //enum type {line, linestrip, arc, circle, bezier, ellipse, spline, rectangular, polygon};

    std::string type; //line, linestrip, etc
    std::string layer;
    double linewidth;
    unsigned short linetype;
    int linescale;
    double red,green,blue,alpha;
    int dxfcolor;

    POINT start;
    POINT end;
    POINT center;
    POINT midpoint;
    POINT input;
    std::vector<POINT> control;
    std::vector<POINT> intersect;
    int end_intersection=-1; //final end intersection number, preprocess data for lib_pocket.h
    int sector; //final sector number, calculated by lib_pocket.h
    std::vector<double> intersect_deg_angle;
    double lenght; //only used calculate and erase dots at line objects
    double pi_angle_start, pi_angle_end, deg_angle_start, deg_angle_end, radius, ratio, sides;
    bool select=0;
    double d; //orginal arc determinant

    //cam section
    int nr=0;
};
extern OBJECT object;
extern std::vector<OBJECT> cad,cam;

struct CONTOUR {
    std::vector<int> sequence; //object sequence inside contour
    std::vector<int> inside; //containing inside contours
    int contourtype=0; //0=open, 1=closed, 2=single object closed
    int depth=0;
    bool select=0;
    double area=0;
};
extern CONTOUR contour;
extern std::vector<CONTOUR> contours;

extern std::vector<int> sequence;

struct SECTOR{
    std::vector<int> primitive;
    double area;
};
extern std::vector<SECTOR> sectors;

#endif // OBJECT_H
