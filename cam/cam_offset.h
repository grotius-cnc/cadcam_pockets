#ifndef FILE_CAM_OFFSET
#define FILE_CAM_OFFSET
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
    DATE      : 14.06.2020
    REVISION  : --
    TODO      : --
    USAGE     : --
    PROTOCOL  : --
    NOTES     : --
*/
#include <iostream>
#include <algorithm>
#include <structs.h>
#include <math.h>
#include <lib/lib_intersect.h>
#include <lib/lib_offset.h>
#include <lib/lib_split.h>
#include <lib/lib_pocket.h>
#include <lib/lib_connect.h>
#include <lib/lib_matrix.h>
#include <lib/lib_convert.h>
#include <CavalierContours/include/cavc/polylineoffset.hpp>
#include <CavalierContours/include/cavc/polylineoffsetislands.hpp>


std::vector<OBJECT> find_intersections(std::vector<OBJECT> array);
OBJECT construct_arc(std::vector<OBJECT>::iterator it, std::vector<OBJECT>::iterator it1, double d);
std::vector<OBJECT> connect_objects(std::vector<OBJECT> array);
OBJECT raw_offset(std::vector<OBJECT>::iterator it);
POINT bulge_to_arc_controlpoint(POINT p1, POINT p2, double bulge);
std::vector<double> arc_bulge(std::vector<OBJECT>::iterator it);

OBJECT raw_offset(std::vector<OBJECT>::iterator it){
    if(it->type=="line"){
        OBJECT obj=line_offset(it,offset);
        obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
        return obj;
    }
    if(it->type=="arc"){
        if(std::round(it->red)==255 && std::round(it->green)==255){ //yellow, inside offset
            OBJECT obj=arc_offset(*it,offset);
            obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
            return obj;
        }
        if(std::round(it->red)==255 && std::round(it->green)==0){ //red, outside offset
            OBJECT obj=arc_offset(*it,abs(offset));
            obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
            return obj;
        }
    }
    if(it->type=="circle"){
        bool checked=true;
        double temp_offset=offset;
        if(temp_offset<0){
            if(abs(temp_offset)>it->radius){ //dont add circles with negative radius
                checked=false;
            }
        }
        if(checked){cam.clear();
            if(std::round(it->red)==255 && std::round(it->green)==255){ //yellow, inside offset
                OBJECT obj;
                obj=circle_offset(it,offset);
                obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
                return obj;
            }
        }
        if(std::round(it->red)==255 && std::round(it->green)==0){ //red, outside offset
            OBJECT obj;
            obj=circle_offset(it,abs(offset));
            obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
            return obj;
        }
    }
    if(it->type=="linestrip" || it->type=="rectangular" || it->type=="polygon"){
        OBJECT obj;
        obj=linestrip_offset(it,offset);
        obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
        return obj;
    }
    if(it->type=="spline"){
        OBJECT obj;
        obj=spline_offset_as_linestrip(it,resolution,offset);
        obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
        return obj;
    }
    if(it->type=="ellipse"){
        OBJECT obj;
        obj=ellipse_offset_as_linestrip(it,resolution,offset);
        obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
        return obj;
    }
    if(it->type=="bezier"){
        OBJECT obj;
        obj=bezier_offset_as_linestrip(it,resolution,offset);
        obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
        return obj;
    }
}


void cam_offset(){

    //normal pocket without islands. use offset -1 or 1.
    std::vector<CONTOUR>::iterator it;
    std::vector<int>::iterator it1;
    std::vector<OBJECT>::iterator it2;
    std::vector<POINT>::iterator it3,it4;


    //copy first offset
    for(it=contours.begin(); it<contours.end(); it++){

        if(it->depth==2){
            cavc::Polyline<double> outerloop;
            for(it1=it->sequence.begin(); it1<it->sequence.end(); it1++){
                int object = *it1;
                it2=cad.begin()+object;

                if(it2->type=="spline"){
                    std::vector<POINT> points=spline_to_points(it2,0.01);
                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
                        outerloop.addVertex(it3->x, it3->y, 0);
                    }
                }
                if(it2->type=="ellipse"){
                    std::vector<POINT> points=ellipse_to_points(it2,0.01);
                    std::reverse(points.begin(),points.end()); //make cw contour
                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
                        outerloop.addVertex(it3->x, it3->y, 0);
                    }
                }
                if(it2->type=="bezier"){
                    std::vector<POINT> points=bezier_to_points(it2,0.01);
                    std::reverse(points.begin(),points.end()); //make cw contour
                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
                        outerloop.addVertex(it3->x, it3->y, 0);
                    }
                }
                if(it2->type=="rectangular"||it2->type=="linestrip"||it2->type=="polygon"){
                    outerloop.addVertex(it2->start.x, it2->start.y, 0); //startpoint
                    std::cout<<"linestrip startpoint inserted"<<std::endl;
                    for(it4=it2->control.begin(); it4<it2->control.end(); it4++){ //controlpoints
                        std::cout<<"linestrip controlpoint inserted"<<std::endl;
                        outerloop.addVertex(it4->x, it4->y, 0);
                    }
                }
                if(it2->type=="arc"){
                    std::vector<double> bulge=arc_bulge(it2);
                    if(bulge[1]==0){ //single arc
                        outerloop.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
                    }
                    if(bulge[1]==1){ //dual arc
                        outerloop.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
                        outerloop.addVertex(it2->control.front().x, it2->control.front().y, bulge[0]); //startpoint arc + bulge
                    }
                }
                if(it2->type=="circle"){
                    outerloop.addVertex(it2->start.x, it2->start.y, -1); //startpoint arc + full bulge=semicircle, -1=g2, 1=g3
                    outerloop.addVertex(it2->control.at(3).x, it2->control.at(3).y, -1); //startpoint arc + bulge
                }
                if(it2->type=="line"){
                    outerloop.addVertex(it2->start.x, it2->start.y, 0);
                }

            }
            outerloop.isClosed() = true;

            //process data
            std::vector<cavc::Polyline<double>> results = cavc::parallelOffset(outerloop, offset);
            //std::vector<cavc::Polyline<double>> results = cavc::parallelOffset(input, offset);
            OBJECT obj;
            for(unsigned int i=0; i<results.size(); i++){ //cw loops
                for(unsigned int j=0; j<results.at(i).vertexes().size()-1; j++){

                    if(results.at(i).vertexes().at(j).bulge()==0){ //line
                        obj.type="line";
                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
                        obj.start.x=results.at(i).vertexes().at(j).x();
                        obj.start.y=results.at(i).vertexes().at(j).y();
                        obj.end.x=results.at(i).vertexes().at(j+1).x();
                        obj.end.y=results.at(i).vertexes().at(j+1).y();
                        cam.push_back(obj);
                    }
                    if(results.at(i).vertexes().at(j).bulge()!=0){ //arc
                        obj.type="arc";
                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
                        obj.start.x=results.at(i).vertexes().at(j).x();
                        obj.start.y=results.at(i).vertexes().at(j).y();
                        obj.end.x=results.at(i).vertexes().at(j+1).x();
                        obj.end.y=results.at(i).vertexes().at(j+1).y();
                        POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.at(i).vertexes().at(j).bulge());
                        obj.control.clear();
                        obj.control.push_back({p.x,p.y,0});
                        cam.push_back(obj);
                    }
                }

                //add last primitive of contour
                if(results.at(i).vertexes().back().bulge()==0){ //line
                    obj.type="line";
                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
                    obj.end.x=results.at(i).vertexes().front().x();
                    obj.end.y=results.at(i).vertexes().front().y();
                    obj.start.x=results.at(i).vertexes().back().x();
                    obj.start.y=results.at(i).vertexes().back().y();
                    cam.push_back(obj);

                }
                if(results.at(i).vertexes().back().bulge()!=0){ //arc
                    obj.type="arc";
                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
                    obj.end.x=results.at(i).vertexes().front().x();
                    obj.end.y=results.at(i).vertexes().front().y();
                    obj.start.x=results.at(i).vertexes().back().x();
                    obj.start.y=results.at(i).vertexes().back().y();
                    POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.at(i).vertexes().back().bulge());
                    obj.control.clear();
                    obj.control.push_back({p.x,p.y,0});
                    cam.push_back(obj);
                }
            }
        }
    }



}


//void cam_offset(){ //pockets with islands. must draw a outside box (depth 0), a inner box (depth 1) with a inner box (depth 2).

//    std::vector<CONTOUR>::iterator it;
//    std::vector<int>::iterator it1;
//    std::vector<OBJECT>::iterator it2;
//    std::vector<POINT>::iterator it3,it4;
//    std::vector<OBJECT> a;

//    //convert cad primitives to cam line and linestrips.

//    cavc::Polyline<double> outerloop;
//    //cavc::Polyline<double> island; //must be clockwise, red is clockwise already!
//    std::vector<cavc::Polyline<double>> islands; //save multiple inner islands

//    //copy first offset
//    for(it=contours.begin(); it<contours.end(); it++){



//        if(it->depth==1){ //outer contour
//             std::cout<<"depth 1 found "<<std::endl;
//            std::vector<int> inner;
//            inner = it->inside;

//            for(int i=0; i<inner.size(); i++){
//                std::cout<<"inner: "<<inner.at(i)<<std::endl;
//            }

//            for(it1=it->sequence.begin(); it1<it->sequence.end(); it1++){
//                int object = *it1;
//                it2=cad.begin()+object;

//                if(it2->type=="spline"){
//                    std::vector<POINT> points=spline_to_points(it2,0.01);
//                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                        outerloop.addVertex(it3->x, it3->y, 0);
//                    }
//                }
//                if(it2->type=="ellipse"){
//                    std::vector<POINT> points=ellipse_to_points(it2,0.01);
//                    std::reverse(points.begin(),points.end()); //make cw contour
//                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                        outerloop.addVertex(it3->x, it3->y, 0);
//                    }
//                }
//                if(it2->type=="bezier"){
//                    std::vector<POINT> points=bezier_to_points(it2,0.01);
//                    std::reverse(points.begin(),points.end()); //make cw contour
//                    for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                        outerloop.addVertex(it3->x, it3->y, 0);
//                    }
//                }
//                if(it2->type=="rectangular"||it2->type=="linestrip"||it2->type=="polygon"){
//                    outerloop.addVertex(it2->start.x, it2->start.y, 0); //startpoint
//                    std::cout<<"linestrip startpoint inserted"<<std::endl;
//                    for(it4=it2->control.begin(); it4<it2->control.end(); it4++){ //controlpoints
//                        std::cout<<"linestrip controlpoint inserted"<<std::endl;
//                        outerloop.addVertex(it4->x, it4->y, 0);
//                    }
//                }
//                if(it2->type=="arc"){
//                    std::vector<double> bulge=arc_bulge(it2);
//                    if(bulge[1]==0){ //single arc
//                        outerloop.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
//                    }
//                    if(bulge[1]==1){ //dual arc
//                        outerloop.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
//                        outerloop.addVertex(it2->control.front().x, it2->control.front().y, bulge[0]); //startpoint arc + bulge
//                    }
//                }
//                if(it2->type=="circle"){
//                    outerloop.addVertex(it2->start.x, it2->start.y, -1); //startpoint arc + full bulge=semicircle, -1=g2, 1=g3
//                    outerloop.addVertex(it2->control.at(3).x, it2->control.at(3).y, -1); //startpoint arc + bulge
//                }
//                if(it2->type=="line"){
//                    outerloop.addVertex(it2->start.x, it2->start.y, 0);
//                }

//            }
//            outerloop.isClosed() = true;

//            //do insides here
//            std::vector<int>::iterator it1;
//            for(it=contours.begin(); it<contours.end(); it++){
//                for(it1=it->inside.begin(); it1<it->inside.end(); it1++){
//                    for(int i=0; i<inner.size(); i++){

//                        if(*it1==inner.at(i)){


//                            if(it->depth==2){ //contour islands
//                                cavc::Polyline<double> island; //must be clockwise, red is clockwise already!
//                                for(it1=it->sequence.begin(); it1<it->sequence.end(); it1++){
//                                    int object = *it1;
//                                    it2=cad.begin()+object;

//                                    if(it2->type=="spline"){
//                                        std::vector<POINT> points=spline_to_points(it2,0.01);
//                                        for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                                            island.addVertex(it3->x, it3->y, 0);
//                                        }
//                                    }
//                                    if(it2->type=="ellipse"){
//                                        std::vector<POINT> points=ellipse_to_points(it2,0.01);
//                                        std::reverse(points.begin(),points.end()); //make cw contour
//                                        for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                                            island.addVertex(it3->x, it3->y, 0);
//                                        }
//                                    }
//                                    if(it2->type=="bezier"){
//                                        std::vector<POINT> points=bezier_to_points(it2,0.01);
//                                        std::reverse(points.begin(),points.end()); //make cw contour
//                                        for(it3=points.begin(); it3<points.end(); it3++){ //we don't need the spline endpoint
//                                            island.addVertex(it3->x, it3->y, 0);
//                                        }
//                                    }
//                                    if(it2->type=="rectangular"||it2->type=="linestrip"||it2->type=="polygon"){
//                                        island.addVertex(it2->start.x, it2->start.y, 0); //startpoint
//                                        std::cout<<"linestrip startpoint inserted"<<std::endl;
//                                        for(it4=it2->control.begin(); it4<it2->control.end(); it4++){ //controlpoints
//                                            std::cout<<"linestrip controlpoint inserted"<<std::endl;
//                                            island.addVertex(it4->x, it4->y, 0);
//                                        }
//                                    }
//                                    if(it2->type=="arc"){
//                                        std::vector<double> bulge=arc_bulge(it2);
//                                        if(bulge[1]==0){ //single arc
//                                            island.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
//                                        }
//                                        if(bulge[1]==1){ //dual arc
//                                            island.addVertex(it2->start.x, it2->start.y, bulge[0]); //startpoint arc + bulge
//                                            island.addVertex(it2->control.front().x, it2->control.front().y, bulge[0]); //startpoint arc + bulge
//                                        }
//                                    }
//                                    if(it2->type=="circle"){
//                                        island.addVertex(it2->start.x, it2->start.y, -1); //startpoint arc + full bulge=semicircle, -1=g2, 1=g3
//                                        island.addVertex(it2->control.at(3).x, it2->control.at(3).y, -1); //startpoint arc + bulge
//                                    }
//                                    if(it2->type=="line"){
//                                        island.addVertex(it2->start.x, it2->start.y, 0);
//                                    }
//                                }
//                                island.isClosed() = true;
//                                islands.push_back(island);
//                            }




//                        }
//                    }
//                }
//            }

//            cavc::OffsetLoopSet<double> loopSet;
//            loopSet.ccwLoops.push_back({0, outerloop, cavc::createApproxSpatialIndex(outerloop)});

//            for(int i=0; i<islands.size(); i++){
//                loopSet.cwLoops.push_back({0, islands.at(i), cavc::createApproxSpatialIndex(islands.at(i))});
//            }


//            //loopSet.cwLoops.push_back({0, island, cavc::createApproxSpatialIndex(island)});
//            // add the surrounding counter clockwise loop
//            // constructed with {parent index, closed polyline, spatial index}
//            // this structure is also returned and can be fed back into the offset algorithm (the spatial
//            // indexes are created by the algorithm and used for the next iteration)
//            // NOTE: parent index can always be 0 (it is just used internally, API likely to be improved in
//            // the future...), spatial index must always be created with the associated polyline


//            // add the clockwise loop islands
//            //loopSet.cwLoops.push_back({0, islands, cavc::createApproxSpatialIndex(islands)});


//            // NOTE: this algorithm requires all polylines to be closed and non-self intersecting, and not
//            // intersect any of the other input polylines
//            cavc::ParallelOffsetIslands<double> alg;
//            // NOTE: offset delta is always taken as an absolute magnitude
//            // (negative values are the same as positive, to change offset direction you can change the
//            // orientation of the loops e.g. from clockwise to counter clockwise)
//            //const double offsetDelta = 1.0;
//            // compute first offset (offset by 1.0)
//            cavc::OffsetLoopSet<double> results = alg.compute(loopSet, offset);
//            // compute offset of first offset result (offset by another 1.0)
//            //cavc::OffsetLoopSet<double> offsetResult2 = alg.compute(offsetResult, offsetDelta);


//            //std::vector<cavc::Polyline<double>> results = cavc::parallelOffset(input, offset);
//            OBJECT obj;
//            for(unsigned int i=0; i<results.cwLoops.size(); i++){ //cw loops
//                for(unsigned int j=0; j<results.cwLoops.at(i).polyline.size()-1; j++){

//                    if(results.cwLoops.at(i).polyline.vertexes().at(j).bulge()==0){ //line
//                        obj.type="line";
//                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                        obj.start.x=results.cwLoops.at(i).polyline.vertexes().at(j).x();
//                        obj.start.y=results.cwLoops.at(i).polyline.vertexes().at(j).y();
//                        obj.end.x=results.cwLoops.at(i).polyline.vertexes().at(j+1).x();
//                        obj.end.y=results.cwLoops.at(i).polyline.vertexes().at(j+1).y();
//                        cam.push_back(obj);
//                    }
//                    if(results.cwLoops.at(i).polyline.vertexes().at(j).bulge()!=0){ //arc
//                        obj.type="arc";
//                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                        obj.start.x=results.cwLoops.at(i).polyline.vertexes().at(j).x();
//                        obj.start.y=results.cwLoops.at(i).polyline.vertexes().at(j).y();
//                        obj.end.x=results.cwLoops.at(i).polyline.vertexes().at(j+1).x();
//                        obj.end.y=results.cwLoops.at(i).polyline.vertexes().at(j+1).y();
//                        POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.cwLoops.at(i).polyline.vertexes().at(j).bulge());
//                        obj.control.clear();
//                        obj.control.push_back({p.x,p.y,0});
//                        cam.push_back(obj);
//                    }
//                }

//                //add last primitive of contour
//                if(results.cwLoops.at(i).polyline.vertexes().back().bulge()==0){ //line
//                    obj.type="line";
//                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                    obj.end.x=results.cwLoops.at(i).polyline.vertexes().front().x();
//                    obj.end.y=results.cwLoops.at(i).polyline.vertexes().front().y();
//                    obj.start.x=results.cwLoops.at(i).polyline.vertexes().back().x();
//                    obj.start.y=results.cwLoops.at(i).polyline.vertexes().back().y();
//                    cam.push_back(obj);

//                }
//                if(results.cwLoops.at(i).polyline.vertexes().back().bulge()!=0){ //arc
//                    obj.type="arc";
//                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                    obj.end.x=results.cwLoops.at(i).polyline.vertexes().front().x();
//                    obj.end.y=results.cwLoops.at(i).polyline.vertexes().front().y();
//                    obj.start.x=results.cwLoops.at(i).polyline.vertexes().back().x();
//                    obj.start.y=results.cwLoops.at(i).polyline.vertexes().back().y();
//                    POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.cwLoops.at(i).polyline.vertexes().back().bulge());
//                    obj.control.clear();
//                    obj.control.push_back({p.x,p.y,0});
//                    cam.push_back(obj);
//                }
//            }

//            for(unsigned int i=0; i<results.ccwLoops.size(); i++){ //ccw loops
//                for(unsigned int j=0; j<results.ccwLoops.at(i).polyline.size()-1; j++){

//                    if(results.ccwLoops.at(i).polyline.vertexes().at(j).bulge()==0){ //line
//                        obj.type="line";
//                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                        obj.start.x=results.ccwLoops.at(i).polyline.vertexes().at(j).x();
//                        obj.start.y=results.ccwLoops.at(i).polyline.vertexes().at(j).y();
//                        obj.end.x=results.ccwLoops.at(i).polyline.vertexes().at(j+1).x();
//                        obj.end.y=results.ccwLoops.at(i).polyline.vertexes().at(j+1).y();
//                        cam.push_back(obj);
//                    }
//                    if(results.ccwLoops.at(i).polyline.vertexes().at(j).bulge()!=0){ //arc
//                        obj.type="arc";
//                        obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                        obj.start.x=results.ccwLoops.at(i).polyline.vertexes().at(j).x();
//                        obj.start.y=results.ccwLoops.at(i).polyline.vertexes().at(j).y();
//                        obj.end.x=results.ccwLoops.at(i).polyline.vertexes().at(j+1).x();
//                        obj.end.y=results.ccwLoops.at(i).polyline.vertexes().at(j+1).y();
//                        POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.ccwLoops.at(i).polyline.vertexes().at(j).bulge());
//                        obj.control.clear();
//                        obj.control.push_back({p.x,p.y,0});
//                        cam.push_back(obj);
//                    }
//                }

//                //add last primitive of contour
//                if(results.ccwLoops.at(i).polyline.vertexes().back().bulge()==0){ //line
//                    obj.type="line";
//                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                    obj.end.x=results.ccwLoops.at(i).polyline.vertexes().front().x();
//                    obj.end.y=results.ccwLoops.at(i).polyline.vertexes().front().y();
//                    obj.start.x=results.ccwLoops.at(i).polyline.vertexes().back().x();
//                    obj.start.y=results.ccwLoops.at(i).polyline.vertexes().back().y();
//                    cam.push_back(obj);

//                }
//                if(results.ccwLoops.at(i).polyline.vertexes().back().bulge()!=0){ //arc
//                    obj.type="arc";
//                    obj.red=0; obj.green=255; obj.blue=0; obj.alpha=255;
//                    obj.end.x=results.ccwLoops.at(i).polyline.vertexes().front().x();
//                    obj.end.y=results.ccwLoops.at(i).polyline.vertexes().front().y();
//                    obj.start.x=results.ccwLoops.at(i).polyline.vertexes().back().x();
//                    obj.start.y=results.ccwLoops.at(i).polyline.vertexes().back().y();
//                    POINT p=bulge_to_arc_controlpoint({obj.start.x,obj.start.y,0},{obj.end.x,obj.end.y,0},results.ccwLoops.at(i).polyline.vertexes().back().bulge());
//                    obj.control.clear();
//                    obj.control.push_back({p.x,p.y,0});
//                    cam.push_back(obj);
//                }
//            }
//        }
//    }
//}

POINT bulge_to_arc_controlpoint(POINT p1, POINT p2, double bulge){ //find the arc center

    //bulge neg=g2
    //bulge pos=g3
    //bulge 0=line
    //bulge 1=semicircle
    //m=(p1+p2)/2
    //Bulge = d1/d2=tan(Delta/4)
    //http://www.lee-mac.com/bulgeconversion.html#bulgearc
    //https://math.stackexchange.com/questions/1337344/get-third-point-from-an-arc-constructed-by-start-point-end-point-and-bulge

    double dist=sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)); //dist=chord lenght p1-p2
    POINT m={(p1.x+p2.x)/2,(p1.y+p2.y)/2,0}; //m=point half chord lenght p1-p2
    double d1=(abs(bulge))*(0.5*dist); //d1=height from m to arc controlpoint

    //normalize p1, rotate around m, bring m back to x,y 0,0
    double p1x_nor=p1.x-m.x;
    double p1y_nor=p1.y-m.y;

    POINT p1_rot;
    if(bulge<0){ //g2
        p1_rot=rotate_3d(p1x_nor,p1y_nor,0,0,0,-90);
    }
    if(bulge>0){ //g3
        p1_rot=rotate_3d(p1x_nor,p1y_nor,0,0,0,90);
    }


    p1_rot.x+=m.x; //bring the rotated p1 back to orginal coordinates
    p1_rot.y+=m.y;

    POINT q=offset_point_on_line(m.x,m.y,p1_rot.x,p1_rot.y,d1);

    //std::cout<<"arc controlpoint x: "<<q.x<<" y: "<<q.y<<std::endl;
    return q; //arc controlpoint

}

std::vector<double> arc_bulge(std::vector<OBJECT>::iterator it){

    //https://github.com/jbuckmccready/CavalierContours/issues/17

    //bulge neg=g2
    //bulge pos=g3
    double pi_angle_arc=0;
    double pi_angle_start=0;
    double pi_angle_end=0;
    std::vector<double> bulge; //bulge[0]=bulge value, bulge[1]=0 (one arc), bulge[1]=1 (two arcs)
    double r=it->radius;
    POINT a=it->start;
    POINT b=it->end;
    POINT c=it->center;

    if(it->d>0){ //g2, d=determinant
        pi_angle_start = atan2(b.y-c.y, b.x-c.x);
        pi_angle_end = atan2(a.y-c.y, a.x-c.x);
    }
    if(it->d<0){ //g3
        pi_angle_start = atan2(a.y-c.y, a.x-c.x);
        pi_angle_end  = atan2(b.y-c.y, b.x-c.x);
    }
    if(it->d==0){
        bulge.push_back(0); //draw straight line
        bulge.push_back(0);
    }

    if(pi_angle_end<pi_angle_start){pi_angle_end+=2*M_PI;}
    pi_angle_arc=pi_angle_end-pi_angle_start;

    if(pi_angle_arc>M_PI){ //split up in 2 arcs
        double half_pi_angle_arc=pi_angle_arc/2;
        double bulges=tan(half_pi_angle_arc/4);
        if(it->d<0){
            bulge.push_back(abs(bulges));
            bulge.push_back(1);
        }
        if(it->d>0){
            bulge.push_back(-abs(bulges));
            bulge.push_back(1);
        }

    } else {
        if(it->d<0){
            bulge.push_back(abs(tan(pi_angle_arc/4))); //keep it as 1 arc
            bulge.push_back(0);
        }
        if(it->d>0){
            bulge.push_back(-abs(tan(pi_angle_arc/4))); //keep it as 1 arc
            bulge.push_back(0);
        }
    }

    return bulge;
}



std::vector<OBJECT> find_intersections(std::vector<OBJECT> array){
    std::vector<OBJECT>::iterator it,it1;
    for(it=array.begin(); it<array.end(); it++){

        for(it1=array.begin(); it1<array.end(); it1++){
            if(it!=it1){

                if(it->type=="line"&&it1->type=="line"){
                    line_line_intersect(it,it1);
                }
                if(it->type=="linestrip"&&it1->type=="linestrip"){
                    linestrip_linestrip_intersect(it,it1);
                }
                if(it->type=="arc"&&it1->type=="arc"){
                    arc_arc_intersect(it,it1);
                }
                if(it->type=="line"&&it1->type=="arc"){
                    line_arc_intersect(it,it1);
                }
                if(it->type=="linestrip"&&it1->type=="arc"){  //is oke
                    linestrip_arc_intersect(it,it1);
                }
                if(it->type=="linestrip"&&it1->type=="line"){
                    linestrip_line_intersect(it,it1);
                }
                if(it->type=="linestrip"){
                    linestrip_self_intersect(it);
                }
            }
        }
    }
    return array;
}

OBJECT construct_arc(std::vector<OBJECT>::iterator it, std::vector<OBJECT>::iterator it1, double d){
    OBJECT obj;
    obj.type="arc";
    obj.red=255; obj.green=0; obj.blue=255; obj.alpha=255;
    obj.start.x=it->end.x;
    obj.start.y=it->end.y;
    obj.end.x=it1->start.x;
    obj.end.y=it1->start.y;
    obj.radius=abs(offset);
    obj.control.resize(1);
    obj.d=d;

    double radsq=obj.radius*obj.radius;
    double q=sqrt(((obj.end.x-obj.start.x)*(obj.end.x-obj.start.x))+((obj.end.y-obj.start.y)*(obj.end.y-obj.start.y)));
    double cx=(obj.start.x+obj.end.x)/2; //center x
    double cy=(obj.start.y+obj.end.y)/2; //center y
    double pi_angle_start=0,pi_angle_end=0;

    if(d>0){ //G2
        obj.center.x=cx-sqrt(radsq-((q/2)*(q/2)))*((obj.start.y-obj.end.y)/q);
        obj.center.y=cy-sqrt(radsq-((q/2)*(q/2)))*((obj.end.x-obj.start.x)/q);

        pi_angle_end=atan2(obj.start.y-obj.center.y, obj.start.x-obj.center.x);
        pi_angle_start=atan2(obj.end.y-obj.center.y, obj.end.x-obj.center.x);
        if(pi_angle_start>pi_angle_end){pi_angle_end+=2*M_PI;}
        obj.control.front().x=obj.center.x+(cos((pi_angle_start+pi_angle_end)/2)*obj.radius);
        obj.control.front().y=obj.center.y+(sin((pi_angle_start+pi_angle_end)/2)*obj.radius);
    }
    if(d<0){ //G3
        obj.center.x=cx+sqrt(radsq-((q/2)*(q/2)))*((obj.start.y-obj.end.y)/q);
        obj.center.y=cy+sqrt(radsq-((q/2)*(q/2)))*((obj.end.x-obj.start.x)/q);

        pi_angle_end=atan2(obj.end.y-obj.center.y, obj.end.x-obj.center.x);
        pi_angle_start=atan2(obj.start.y-obj.center.y, obj.start.x-obj.center.x);
        if(pi_angle_start>pi_angle_end){pi_angle_end+=2*M_PI;}
        obj.control.front().x=obj.center.x+(cos((pi_angle_start+pi_angle_end)/2)*obj.radius);
        obj.control.front().y=obj.center.y+(sin((pi_angle_start+pi_angle_end)/2)*obj.radius);
    }
    return obj;
}

#endif





