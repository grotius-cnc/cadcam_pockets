#include "variable.h"
#include <string>
#include <vector>
#include <iostream>
#include <opengl.h>

int click,int_key;
double zl,zr,zt,zb,zf,xr,yr,pxw;
double mx,my,mz; //mouse cad
double mx_snap,my_snap,mz_snap; //mouse cad snap
double aspect;
double segments = 25;

std::string message;
std::string mode;
int progressbar_value;

double offset;
double resolution=0.05; //spline to linestrip conversion resolution, same idea is used for ellipse, bezier etc.
bool inside_offset;
bool outside_offset;

variable::variable()
{

}
























