#ifndef VARIABLE_H
#define VARIABLE_H
#include <vector>
#include <string>

extern int click,int_key;
extern double zl,zr,zt,zb,zf,xr,yr,pxw;
extern double mx,my,mz; //mouse cad
extern double mx_snap,my_snap,mz_snap; //mouse cad snap
extern double aspect;
extern double segments;

extern std::string message;
extern std::string mode;
extern int progressbar_value;

extern double offset;
extern double resolution;
extern bool inside_offset;
extern bool outside_offset;

class variable
{
public:
    variable();
};

#endif // VARIABLE_H
