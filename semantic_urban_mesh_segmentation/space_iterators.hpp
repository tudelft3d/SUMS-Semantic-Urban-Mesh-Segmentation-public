/*
*   Name        : space_iterators.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh sampling iterator
*   Availability:
*   Copyright   : Copyright (C) 2021 by Weixiao GAO (gaoweixiaocuhk@gmail.com)
*                 All rights reserved.
*
*				  This file is part of semantic_urban_mesh_segmentation: software
*				  for semantic segmentation of textured urban meshes.
*
*				  semantic_urban_mesh_segmentation is free software; you can
*				  redistribute it and/or modify it under the terms of the GNU
*				  General Public License Version 3 as published by the Free
*				  Software Foundation.
*
*				  semantic_urban_mesh_segmentation is distributed in the hope that
*				  it will be useful, but WITHOUT ANY WARRANTY; without even the
*				  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
*				  PURPOSE. See the GNU General Public License for more details.
*
*				  You should have received a copy of the GNU General Public License
*				  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#ifndef semantic_mesh_segmentation__SPACE_ITERATORS_HPP
#define semantic_mesh_segmentation__SPACE_ITERATORS_HPP

#include <vector>
#include <easy3d/point_cloud.h>
#include "math_base.hpp"

namespace semantic_mesh_segmentation
{
    template <class Spatial_Idexing,class INTFUNCTOR,class TMARKER>
    class RayIterator
    {
    public:
		typedef typename Spatial_Idexing::ScalarType ScalarType;
		typedef typename vcg::Ray3<ScalarType> RayType;
		typedef typename Spatial_Idexing::Box3x IndexingBoxType;
    protected:
		typedef typename Spatial_Idexing::ObjType ObjType;
		typedef typename easy3d::Vec<3, ScalarType>  CoordType;
		typedef typename Spatial_Idexing::CellIterator CellIterator;
            ScalarType max_dist;

            void _ControlLimits()
            {
				for (int i = 0; i < 3; i++)
				{
					easy3d::Vec<3, int> dim = Si.siz;
					if (CurrentCell[i] < 0)
						CurrentCell[i] = 0;
					else
						if (CurrentCell[i] >= dim[i])
								CurrentCell[i] = dim[i] - 1;
				}
            }

            void _FindLinePar()
            {
                /* Punti goal */
				easy3d::Vec<3, int> ip;
				Si.PToIP(start, ip);
				Si.IPiToPf(ip, goal);
				for (int i = 0; i < 3; i++)
					if (r.Direction()[i] > 0.0)
						goal[i] += Si.voxel[i];

				ScalarType gx = goal.x();
				ScalarType gy = goal.y();
				ScalarType gz = goal.z();

				dist = (r.Origin() - goal).norm();

				const float LocalMaxScalar = (std::numeric_limits<float>::max)();
                const float EPS = std::numeric_limits<float>::min();

                /* Parametri della linea */
                ScalarType tx,ty,tz;

                if(fabs(r.Direction().x())>EPS)
					tx = (gx-r.Origin().x())/r.Direction().x();
                else
					tx = LocalMaxScalar;

                if(fabs(r.Direction().y())>EPS)
					ty = (gy - r.Origin().y()) / r.Direction().y();
                else
					ty = LocalMaxScalar;

                if(fabs(r.Direction().z())>EPS)
                    tz = (gz - r.Origin().z())/r.Direction().z();
                else
                    tz = LocalMaxScalar;

                t = CoordType(tx,ty,tz);
            }

            bool _controlEnd()
            {
                return  (((CurrentCell.x()<0)||(CurrentCell.y()<0)||(CurrentCell.z()<0))||
                            ((CurrentCell.x()>=Si.siz.x())||(CurrentCell.y()>=Si.siz.y())||(CurrentCell.z()>=Si.siz.z())));
            }

			// line-box
			bool IntersectionLineBox(const easy3d::GenericBox3<ScalarType> &box, const Line3<ScalarType> &r, easy3d::Vec<3, ScalarType> & coord )
			{
				const int NUMDIM = 3;
				const int RIGHT  = 0;
				const int LEFT   = 1;
				const int MIDDLE = 2;
	
				int inside = 1;
				char quadrant[NUMDIM];
				int i;
				int whichPlane;
				easy3d::Vec<3, ScalarType> maxT,candidatePlane;
	    
					// Find candidate planes; this loop can be avoided if
					// rays cast all from the eye(assume perpsective view)
				for (i=0; i<NUMDIM; i++)
				{
					if(r.Origin()[i] < box.min[i])
					{
						quadrant[i] = LEFT;
						candidatePlane[i] = box.min[i];
						inside = 0;
					}
					else if (r.Origin()[i] > box.max[i])
					{
						quadrant[i] = RIGHT;
						candidatePlane[i] = box.max[i];
						inside = 0;
					}
					else
					{
						quadrant[i] = MIDDLE;
					}
				}
	
				// Ray origin inside bounding box
				if(inside){
					coord = r.Origin();
					return true;
				}
	
				// Calculate T distances to candidate planes 
				for (i = 0; i < NUMDIM; i++)
				{
					if (quadrant[i] != MIDDLE && r.Direction()[i] != 0.)
						maxT[i] = (candidatePlane[i] - r.Origin()[i]) / r.Direction()[i];
					else
						maxT[i] = -1.;
				}
	
					// Get largest of the maxT's for final choice of intersection
				whichPlane = 0;
				for (i = 1; i < NUMDIM; i++)
					if (maxT[whichPlane] < maxT[i])
						whichPlane = i;
	
					// Check final candidate actually inside box 
				if (maxT[whichPlane] < 0.) return false;
				for (i = 0; i < NUMDIM; i++)
					if (whichPlane != i)
					{
						coord[i] = r.Origin()[i] + maxT[whichPlane] * r.Direction()[i];
						if (coord[i] < box.min[i] || coord[i] > box.max[i])
							return false;
					}
					else
					{
						coord[i] = candidatePlane[i];
					}
				return true;                        // ray hits box
			}       

			bool IntersectionRayBox(const easy3d::GenericBox3<ScalarType> & box, const Ray3<ScalarType> & r, easy3d::Vec<3, ScalarType> &coord)
			{
				Line3<ScalarType> l;
				l.SetOrigin(r.Origin());
				l.SetDirection(r.Direction());
				return(IntersectionLineBox<ScalarType>(box, l, coord));
			}

            void _NextCell()
            {
                assert(!end);
                easy3d::GenericBox3<ScalarType> bb_current;

                Si.IPiToPf(CurrentCell,bb_current.min());
                Si.IPiToPf(CurrentCell + easy3d::Vec<3, int>(1,1,1),bb_current.max());

                CoordType inters;
                IntersectionRayBox(bb_current,r,inters);
                ScalarType testmax_dist=(inters-r.Origin()).norm();

                if (testmax_dist>max_dist)
                    end=true;
                else
                {
					if( t.x()<t.y() && t.x()<t.z() )
					{
						if (r.Direction().x() < 0.0)
						{
							goal.x() -= Si.voxel.x(); --CurrentCell.x();
						}
						else
						{
							goal.x() += Si.voxel.x(); ++CurrentCell.x();
						}
						t.x() = (goal.x() - r.Origin().x()) / r.Direction().x();
					}
					else if( t.y()<t.z() )
					{
							if(r.Direction().y()<0.0)
							{goal.y() -= Si.voxel.y(); --CurrentCell.y();}
							else
							{goal.y() += Si.voxel.y(); ++CurrentCell.y();}
							t.y() = (goal.y()-r.Origin().y())/r.Direction().y();
					} else
					{
							if(r.Direction().z()<0.0)
							{ goal.z() -= Si.voxel.z(); --CurrentCell.z();}
							else
							{ goal.z() += Si.voxel.z(); ++CurrentCell.z();}
							t.z() = (goal.z()-r.Origin().z())/r.Direction().z();
					}

					dist = (r.Origin() - goal).norm();
					end = _controlEnd();
				}
            }

    public:
		RayIterator(Spatial_Idexing &_Si,
			INTFUNCTOR _int_funct
			, const ScalarType &_max_dist)
			:Si(_Si), int_funct(_int_funct)
		{
			max_dist = _max_dist;
		};

		void SetMarker(TMARKER _tm)
		{
			tm = _tm;
		}

        void Init(const RayType _r)
        {
            r=_r;
            end=false;
            tm.UnMarkAll();
            Elems.clear();
            //CoordType ip;
            //control if intersect the bounding box of the mesh
            if (Si.bbox.IsIn(r.Origin()))
                    start=r.Origin();
            else
				if (!(IntersectionRayBox<ScalarType>(Si.bbox,r,start)))
				{
					end = true;
					return;
				}

			Si.PToIP(start,CurrentCell);
			_ControlLimits();
			_FindLinePar();
			//go to first intersection
			while ((!End())&& Refresh())
					_NextCell();

        }

		bool End()
		{
			return end;
		}


            bool Refresh()
            {
                    //Elems.clear();

    typename Spatial_Idexing::CellIterator first,last,l;

                    Si.Grid(CurrentCell.X(),CurrentCell.Y(),CurrentCell.Z(),first,last);
                    for(l=first;l!=last;++l)
                    {
                            ObjType* elem=&(*(*l));
                            ScalarType t;
                            CoordType Int;
                            if((!elem->IsD())&&(!tm.IsMarked(elem))&&(int_funct((**l),r,t))&&(t<=max_dist))
                            {
                                    Int=r.Origin()+r.Direction()*t;
                                    Elems.push_back(Entry_Type(elem,t,Int));
                                    tm.Mark(elem);
                            }
                    }
                    std::sort(Elems.begin(),Elems.end());
                    CurrentElem=Elems.rbegin();

                    return((Elems.size()==0)||(Dist()>dist));
            }

            void operator ++()
            {
                    if (!Elems.empty()) Elems.pop_back();

                    CurrentElem = Elems.rbegin();

                    if (Dist()>dist)
                    {
                            if (!End())
                            {
                                    _NextCell();
                                    while ((!End())&&Refresh())
                                            _NextCell();
                            }
                    }
            }

            ObjType &operator *(){return *((*CurrentElem).elem);}

            CoordType IntPoint()
            {return ((*CurrentElem).intersection);}

            ScalarType Dist()
            {
                    if (Elems.size()>0)
                            return ((*CurrentElem).dist);
                    else
                            return ((ScalarType)FLT_MAX);
            }

            void SetIndexStructure(Spatial_Idexing &_Si)
            {Si=_Si;}



    protected:

            struct Entry_Type
            {
            public:

                    Entry_Type(ObjType* _elem,ScalarType _dist,CoordType _intersection)
                    {
                            elem=_elem;
                            dist=_dist;
                            intersection=_intersection;
                    }
                    inline bool operator <  ( const Entry_Type & l ) const{return (dist > l.dist); }
                    ObjType* elem;
                    ScalarType dist;
                    CoordType intersection;
            };

            RayType r;                                                      //ray to find intersections
            Spatial_Idexing &Si;      //reference to spatial index algorithm
            bool end;                                                               //true if the scan is terminated
            INTFUNCTOR &int_funct;
            TMARKER tm;

            std::vector<Entry_Type> Elems;                                  //element loaded from curren cell
            typedef typename std::vector<Entry_Type>::reverse_iterator ElemIterator;
            ElemIterator CurrentElem;       //iterator to current element

            easy3d::Vec<3, int> CurrentCell;                                               //current cell

            //used for raterization
            CoordType start;
            CoordType goal;
            ScalarType dist;
            CoordType t;

    };


        template <class Spatial_Idexing,class DISTFUNCTOR,class TMARKER>
        class ClosestIterator
        {
                typedef typename Spatial_Idexing::ObjType ObjType;
                typedef typename Spatial_Idexing::ScalarType ScalarType;
                typedef typename vcg::Point3<ScalarType>  CoordType;
                typedef typename Spatial_Idexing::CellIterator CellIterator;



                bool  _EndGrid()
                {
                        if ((explored.min==vcg::Point3i(0,0,0))&&(explored.max==Si.siz))
                                end =true;
                        return end;
                }

                void _UpdateRadius()
                {
                        if (radius>=max_dist)
                                end=true;

                        radius+=step_size;
                        //control bounds
                        if (radius>max_dist)
                                radius=max_dist;
                }

                bool _NextShell()
                {

                        //then expand the box
                        explored=to_explore;
                        _UpdateRadius();
                        Box3<ScalarType> b3d(p,radius);
                        Si.BoxToIBox(b3d,to_explore);
                        Box3i ibox(Point3i(0,0,0),Si.siz-Point3i(1,1,1));
                        to_explore.Intersect(ibox);
                        if (!to_explore.IsNull())
                        {
                                assert(!( to_explore.min.X()<0 || to_explore.max.X()>=Si.siz[0] ||
                                        to_explore.min.Y()<0 || to_explore.max.Y()>=Si.siz[1] ||  to_explore.min.Z()<0
                                        || to_explore.max.Z()>=Si.siz[2] ));
                                return true;
                        }
                        return false;
                }



        public:

                ClosestIterator(Spatial_Idexing &_Si,DISTFUNCTOR _dist_funct):Si(_Si),dist_funct(_dist_funct){}

                void SetIndexStructure(Spatial_Idexing &_Si)
                {Si=_Si;}

                void SetMarker(TMARKER _tm)
                {
                        tm=_tm;
                }

                void Init(CoordType _p,const ScalarType &_max_dist)
                {
                        explored.SetNull();
                        to_explore.SetNull();
                        p=_p;
                        max_dist=_max_dist;
                        Elems.clear();
                        end=false;
                        tm.UnMarkAll();
                        //step_size=Si.voxel.X();
                        step_size=Si.voxel.Norm();
                        radius=0;

                        while ((!_NextShell())&&(!End())) {}

                        while ((!End())&& Refresh()&&(!_EndGrid()))
                                        _NextShell();
                }

                //return true if the scan is complete
                bool End()
                {return end;}

                //and object comes from previos that are already in     the     stack
                //return false if no elements find
                bool Refresh()
                {
                        int     ix,iy,iz;
                        for( iz = to_explore.min.Z();iz <=      to_explore.max.Z(); ++iz)
                                for(iy =to_explore.min.Y(); iy  <=to_explore.max.Y(); ++iy)
                                        for(ix =to_explore.min.X(); ix  <= to_explore.max.X();++ix)
                                        {
                                                // this test is to avoid to re-process already analyzed cells.
                                                if((explored.IsNull())||
                                                        (ix<explored.min[0] || ix>explored.max[0] ||
                                                        iy<explored.min[1] || iy>explored.max[1] ||
                                                        iz<explored.min[2] || iz>explored.max[2] ))
                                                {
                                                        typename Spatial_Idexing::CellIterator first,last,l;

                                                        Si.Grid(ix,iy,iz,first,last);
                                                        for(l=first;l!=last;++l)
                                                        {
                                                                ObjType *elem=&(**l);
                                                                if (!tm.IsMarked(elem))
                                                                {

                                                                        CoordType nearest;
                                                                        ScalarType dist=max_dist;
                                                                        if (dist_funct((**l),p,dist,nearest))
                                                                                Elems.push_back(Entry_Type(elem,fabs(dist),nearest));
                                                                        tm.Mark(elem);
                                                                }
                                                        }
                                                }

                                        }

                                std::sort(Elems.begin(),Elems.end());
                                CurrentElem=Elems.rbegin();

                        return((Elems.size()==0)||(Dist()>radius));
                }

                bool ToUpdate()
                {return ((Elems.size()==0)||(Dist()>radius));}

                void operator ++()
                {
                        if (!Elems.empty()) Elems.pop_back();

                        CurrentElem = Elems.rbegin();

                        if ((!End())&& ToUpdate())
                                do{_NextShell();}
                                        while (Refresh()&&(!_EndGrid()));
                }

                ObjType &operator *(){return *((*CurrentElem).elem);}

                //return distance of the element form the point if no element
                //are in the vector then return max dinstance
                ScalarType Dist()
                {
                        if (Elems.size()>0)
                                return ((*CurrentElem).dist);
                        else
                                return ((ScalarType)FLT_MAX);
                }

                CoordType NearestPoint()
                {return ((*CurrentElem).intersection);}

        protected:

                struct Entry_Type
                {
                public:

                        Entry_Type(ObjType* _elem,ScalarType _dist,CoordType _intersection)
                        {
                                elem=_elem;
                                dist=_dist;
                                intersection=_intersection;
                        }

                        inline bool operator <  ( const Entry_Type & l ) const{return (dist > l.dist); }

                        inline bool operator ==  ( const Entry_Type & l ) const{return (elem == l.elem); }

                        ObjType* elem;
                        ScalarType dist;
                        CoordType intersection;
                };

                CoordType p;                                                    //initial point
                Spatial_Idexing &Si;              //reference to spatial index algorithm
                bool end;                                                                       //true if the scan is terminated
                ScalarType max_dist;              //max distance when the scan terminate
                vcg::Box3i explored;              //current bounding box explored
                vcg::Box3i to_explore;          //current bounding box explored
                ScalarType radius;                        //curret radius for sphere expansion
                ScalarType step_size;             //radius step
                std::vector<Entry_Type> Elems; //element loaded from the current sphere

                DISTFUNCTOR &dist_funct;
                TMARKER tm;

                typedef typename std::vector<Entry_Type>::reverse_iterator ElemIterator;
                ElemIterator CurrentElem;       //iterator to current element

};

}
#endif