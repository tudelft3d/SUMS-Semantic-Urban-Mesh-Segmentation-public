/*
*   Name        : spatial_hashing.hpp
*   Author      : Weixiao GAO
*   Date        : 15/09/2021
*   Version     : 1.0
*   Description : for mesh sampling spatial hashing
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
#ifndef semantic_mesh_segmentation__SPATIAL_HASHING_HPP
#define semantic_mesh_segmentation__SPATIAL_HASHING_HPP

#include <vector>
#include <algorithm>
#include <easy3d/point_cloud.h>
#include <unordered_map>
#define __int64 long long
#define __cdecl 
#define STDEXT stdext

namespace semantic_mesh_segmentation {

	class BasicGrid
	{
	public:

		typedef easy3d::GenericBox3<float> Box3x;
		typedef easy3d::Vec<3, float> CoordType;
		typedef BasicGrid GridType;

		Box3x bbox;

		CoordType dim;
		easy3d::Vec<3, int> siz;
		CoordType voxel;

		/*
		 Derives the right values of Dim and voxel starting
		 from the current values of siz and bbox
		*/
		void ComputeDimAndVoxel()
		{
			this->dim = this->bbox.max() - this->bbox.min();
			this->voxel[0] = this->dim[0] / this->siz[0];
			this->voxel[1] = this->dim[1] / this->siz[1];
			this->voxel[2] = this->dim[2] / this->siz[2];
		}
		/* Given a 3D point, returns the coordinates of the cell where the point is
		 * @param p is a 3D point
		 * @return integer coordinates of the cell
		 */
		inline easy3d::Vec<3, int> GridP(const easy3d::Vec<3, float> & p) const
		{
			easy3d::Vec<3, int> pi;
			PToIP(p, pi);
			return pi;
		}

		/* Given a 3D point p, returns the index of the corresponding cell
		 * @param p is a 3D point in the space
		 * @return integer coordinates pi of the cell
		 */
		inline void PToIP(const CoordType & p, easy3d::Vec<3, int> &pi) const
		{
			CoordType t = p - bbox.min();
			pi[0] = int(t[0] / voxel[0]);
			pi[1] = int(t[1] / voxel[1]);
			pi[2] = int(t[2] / voxel[2]);
		}

		/* Given a cell index return the lower corner of the cell
		 * @param integer coordinates pi of the cell
		 * @return p is a 3D point representing the lower corner of the cell
		 */
		inline void IPiToPf(const easy3d::Vec<3, int> &pi, CoordType &p) const
		{
			p[0] = ((float)pi[0])*voxel[0];
			p[1] = ((float)pi[1])*voxel[1];
			p[2] = ((float)pi[2])*voxel[2];
			p += bbox.min();
		}

		/* Given a cell index return the corresponding box
		 * @param integer coordinates pi of the cell
		 * @return b is the corresponding box in <ScalarType> coordinates
		 */
		inline void IPiToBox(const easy3d::Vec<3, int> & pi, Box3x & b) const
		{
			CoordType p;
			p[0] = ((float)pi[0])*voxel[0];
			p[1] = ((float)pi[1])*voxel[1];
			p[2] = ((float)pi[2])*voxel[2];
			p += bbox.min();

			b = Box3x(p, p + voxel);
		}

		/* Given a cell index return the center of the cell itself
		 * @param integer coordinates pi of the cell
		 * @return b is the corresponding box in <ScalarType> coordinates
		 */inline void IPiToBoxCenter(const easy3d::Vec<3, int> &pi, CoordType & c) const
		{
			CoordType p;
			IPiToPf(pi, p);
			c = p + voxel / float(2.0);
		}

		// Same of IPiToPf but for the case that you just want to transform 
		// from a space to the other.
		inline void IPfToPf(const CoordType & pi, CoordType &p) const
		{
			p[0] = ((float)pi[0])*voxel[0];
			p[1] = ((float)pi[1])*voxel[1];
			p[2] = ((float)pi[2])*voxel[2];
			p += bbox.min();
		}

		/* Given a cell in <ScalarType> coordinates, compute the corresponding cell in integer coordinates
		 * @param b is the cell in <ScalarType> coordinates
		 * @return ib is the correspondent box in integer coordinates
		 */
		void BoxToIBox(const Box3x & b, easy3d::GenericBox3<int> &ib) const //Used
		{
			easy3d::Vec<3, int> coord_min, coord_max;
			coord_min = ib.min(); coord_max = ib.max();

			PToIP(b.min(), coord_min);
			PToIP(b.max(), coord_max);

			ib = easy3d::GenericBox3<int>(coord_min, coord_max);
			//assert(ib.max[0]>=0 && ib.max[1]>=0 && ib.max[2]>=0); 
		}

		/* Given a cell in integer coordinates, compute the corresponding cell in <ScalarType> coordinates
		 * @param ib is the cell in integer coordinates
		 * @return b is the correspondent box in <ScalarType> coordinates
		 */
		void IBoxToBox(const easy3d::GenericBox3<int> & ib, Box3x & b) const
		{
			easy3d::Vec<3, float> coord_min, coord_max;
			coord_min = b.min(); coord_max = b.max();

			IPiToPf(ib.min(), coord_min);
			IPiToPf(ib.max(), coord_max);

			b = Box3x(coord_min, coord_max);
		}
	};

	template<class scalar_type>
	void BestDim(const easy3d::GenericBox3<scalar_type> box, const scalar_type voxel_size, easy3d::Vec<3 ,int> &dim )
	{
		easy3d::Vec<3, scalar_type> box_size = box.max() - box.min();
		__int64 elem_num = (__int64)(box_size[0] / voxel_size + 0.5) *(__int64)(box_size[1] / voxel_size + 0.5) * (__int64)(box_size[2] / voxel_size + 0.5);
		BestDim(elem_num, box_size, dim);
	}  

	template<class scalar_type>
	void BestDim( const __int64 elems, const easy3d::Vec<3, scalar_type> & size, easy3d::Vec<3, int> & dim )
	{
	    const __int64 mincells   = 1;           // Numero minimo di celle
	    const double GFactor = 1;       // GridEntry = NumElem*GFactor
	    double diag = size.norm();      // Diagonale del box
	    double eps  = diag*1e-4;                // Fattore di tolleranza
	
	    assert(elems>0);
	    assert(size[0]>=0.0);
	    assert(size[1]>=0.0);
	    assert(size[2]>=0.0);
	
	    __int64 ncell = (__int64)(elems*GFactor);       // Calcolo numero di voxel
	    if(ncell<mincells)
	            ncell = mincells;
	
	    dim[0] = 1;
	    dim[1] = 1;
	    dim[2] = 1;
	
	    if(size[0]>eps)
	    {
			if (size[1] > eps)
			{
				if (size[2] > eps)
				{
					double k = pow((double)(ncell / (size[0] * size[1] * size[2])), double(1.0 / 3.f));
					dim[0] = int(size[0] * k);
					dim[1] = int(size[1] * k);
					dim[2] = int(size[2] * k);
				}
				else
				{
					dim[0] = int(::sqrt(ncell*size[0] / size[1]));
					dim[1] = int(::sqrt(ncell*size[1] / size[0]));
				}
			}
			else
			{
				if (size[2] > eps)
				{
					dim[0] = int(::sqrt(ncell*size[0] / size[2]));
					dim[2] = int(::sqrt(ncell*size[2] / size[0]));
				}
				else
					dim[0] = int(ncell);
			}
	    }
	    else
	    {
			if (size[1] > eps)
			{
				if (size[2] > eps)
				{
					dim[1] = int(::sqrt(ncell*size[1] / size[2]));
					dim[2] = int(::sqrt(ncell*size[2] / size[1]));
				}
				else
					dim[1] = int(ncell);
			}
			else if (size[2] > eps)
				dim[2] = int(ncell);
	    }

		dim[0] = dim[0] > 1 ? dim[0] : 1;
	    dim[1] = dim[1] > 1 ? dim[1] : 1;
	    dim[2] = dim[2] > 1 ? dim[2] : 1;
	}

	template <class OBJTYPE>
	class SpatialIndex {
	public:
	    typedef SpatialIndex<OBJTYPE> ClassType;
	    typedef OBJTYPE ObjType;
	    typedef ObjType * ObjPtr;
	    typedef easy3d::Vec<3, float> CoordType;
	    typedef easy3d::GenericBox3<float> BoxType;
	
		template <class OBJITER>
		void Set(const OBJITER & _oBegin, const OBJITER & _oEnd)
		{
			assert(0);      // this is a base interface.
			(void)_oBegin;  // avoid "unreferenced parameter" compiler warning.
			(void)_oEnd;
		}
	};
	
    // hashing function
    struct HashFunctor : public std::unary_function<easy3d::Vec<3, int>, size_t>
    {
        enum
        { // parameters for hash table
            bucket_size = 4, // 0 < bucket_size
            min_buckets = 8
        };

        size_t operator()(const easy3d::Vec<3, int> &p) const
        {
			const size_t _HASH_P0 = 73856093u;
			const size_t _HASH_P1 = 19349663u;
			const size_t _HASH_P2 = 83492791u;

			return size_t(p[0])*_HASH_P0 ^  size_t(p[1])*_HASH_P1 ^  size_t(p[2])*_HASH_P2;
        }

        bool operator()(const easy3d::Vec<3, int> &s1, const easy3d::Vec<3, int> &s2) const
        { // test if s1 ordered before s2
            return (s1 < s2);
        }
    };

    template < typename ObjType>
    class SpatialHashTable:public BasicGrid, public SpatialIndex<ObjType>
    {
		public:
		typedef SpatialHashTable SpatialHashType;
		typedef ObjType* ObjPtr;
		typedef easy3d::Vec<3, float> CoordType;
		typedef typename BasicGrid::Box3x Box3x;

		// Hash table definition
		// the hash index directly the grid structure. 
		// We use a MultiMap because we need to store many object (faces) inside each cell of the grid. 

		typedef typename std::unordered_multimap<easy3d::Vec<3, int>, ObjType *, HashFunctor> HashType;
		typedef typename HashType::iterator HashIterator;
		HashType hash_table; // The real HASH TABLE **************************************
        
		// This vector is just a handy reference to all the allocated cells,
		// because hashed multimaps does not expose a direct list of all the different keys. 
		std::vector<easy3d::Vec<3, int>> AllocatedCells;
                
		// Class to abstract a HashIterator (that stores also the key, 
		// while the interface of the generic spatial indexing need only simple object (face) pointers.
        
		struct CellIterator
		{
				CellIterator(){}
				HashIterator t;
				ObjPtr &operator *(){return ((t->second)); }
				ObjPtr operator *() const {return (t->second); }
				bool operator != (const CellIterator & p) const {return t!=p.t;}
				void operator ++() {t++;}

				//easy3d::Vec<3, float> get_result()
				//{
				//	return t->second;
				//}
		};
        
		size_t CellSize(const easy3d::Vec<3, int> &cell)
		{
			return hash_table.count(cell);
		}
                
		inline bool EmptyCell(const easy3d::Vec<3, int> &cell) const
		{
			//for (auto fi : AllocatedCells)
			//{
			//	if (cell.x == fi.x
			//		&& cell.y == fi.y
			//		&& cell.z == fi.z)
			//	{
			//		return false;
			//	}

			//}
			//return true;
			//auto pos = hash_table.find(cell);
			return hash_table.find(cell) == hash_table.end();
		}

		void UpdateAllocatedCells() //Used
		{
			AllocatedCells.clear();
			if (hash_table.empty()) return;
			AllocatedCells.push_back(hash_table.begin()->first);
			for (HashIterator fi = hash_table.begin(); fi != hash_table.end(); ++fi)
			{
				if (AllocatedCells.back() != fi->first) 
					AllocatedCells.push_back(fi->first);
			}
		}
	protected:

		void InsertObject(ObjType *s, const easy3d::Vec<3, int> &cell) //Used
		{
			//if(hash_table.count(cell)==0) AllocatedCells.push_back(cell);
			hash_table.insert(typename HashType::value_type(cell, s));
		}

		void RemoveCell(const easy3d::Vec<3, int> &cell)
		{
		}


		bool RemoveObject(ObjType& s, const easy3d::Vec<3, int> &cell)
		{
			std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(cell);
			CellIterator first; first.t=CellRange.first;
			CellIterator end; end.t=CellRange.second;
			for(CellIterator ci = first; ci!=end;++ci)
			{
				if (*ci == s)
					{
						hash_table.erase(ci.t);
						return true;
					}
			}
			return false;
		}

    public:

        easy3d::GenericBox3<int> Add(ObjType *s) //Used
        {
            easy3d::GenericBox3<float> b;
			b.add_point(*s);
			easy3d::GenericBox3<int> bb;
            BoxToIBox(b,bb);
            //then insert all the cell of bb
            for (int i=bb.x_min();i <= bb.x_max();i++)
				for (int j = bb.y_min(); j <= bb.y_max(); j++)
					for (int k = bb.z_min(); k <= bb.z_max(); k++)
						InsertObject(s, easy3d::Vec<3, int>(i, j, k));

            return bb;
        }

		// it removes s too.
		bool RemoveCell(ObjType &s)
		{
			easy3d::Vec<3, int> pi;
			PToIP(s, pi);
			std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(pi);
			hash_table.erase(CellRange.first,CellRange.second);
			return true;
		}    

		int RemoveInSphere(const easy3d::Vec<3, float> &p, const float radius)
		{
			//Box3x b(p - easy3d::vec3(radius,radius,radius), p + easy3d::vec3(radius,radius,radius));
			//easy3d::GenericBox3<int> bb;
			//BoxToIBox(b,bb);
			//float r2=radius*radius;
			//int cnt=0;
			//std::vector<HashIterator> toDel;

			//for (int i = bb.x_min(); i <= bb.x_max(); i++)
			//	for (int j = bb.y_min(); j <= bb.y_max(); j++)
			//		for (int k = bb.z_min(); k <= bb.z_max(); k++)
			//		{
			//			std::pair<HashIterator, HashIterator> CellRange = hash_table.equal_range(easy3d::Vec<3, int>(i, j, k));
			//			for (HashIterator hi = CellRange.first; hi != CellRange.second; ++hi)
			//			{
			//				if (easy3d::distance2(p, *(hi->second)) <= r2)
			//				{
			//					cnt++;
			//					toDel.push_back(hi);
			//				}
			//			}
			//		}
			//for (typename std::vector<HashIterator>::iterator vi = toDel.begin(); vi != toDel.end(); ++vi)
			//	hash_table.erase(*vi);
			//return cnt;

			std::vector<HashIterator> inSphVec;
			CountInSphere(p, radius, inSphVec);

			for(typename std::vector<HashIterator>::iterator vi= inSphVec.begin(); vi!= inSphVec.end();++vi)
				hash_table.erase(*vi);

			return inSphVec.size();
		}

				// Thsi version of the removal is specialized for the case where
				// an object has a pointshaped box and using the generic bbox interface is just a waste of time.
                
		void RemovePunctual( ObjType &s)
		{
			easy3d::Vec<3, int> pi;
			PToIP(s,pi);
			std::pair<HashIterator,HashIterator> CellRange = hash_table.equal_range(pi);
			for(HashIterator hi = CellRange.first; hi!=CellRange.second;++hi)
			{
				if (hi->second == s)
					{
						hash_table.erase(hi);
						return;
					}
			}
		}

		void Remove( ObjType &s)
		{
			easy3d::GenericBox3<float> b;
			b.add_point(s);
			easy3d::GenericBox3<int> bb;
			BoxToIBox(b,bb);
			//then remove the obj from all the cell of bb
			for (int i=bb.x_min();i<=bb.x_max();i++)
				for (int j=bb.y_min();j<=bb.y_max();j++)
					for (int k=bb.z_min();k<=bb.z_max();k++)
						RemoveObject(s, easy3d::Vec<3, int>(i,j,k));
		}
        
		
		void InitEmpty(const Box3x &_bbox, easy3d::Vec<3, int> grid_size)//Used
		{
				Box3x b;
				Box3x &bbox = this->bbox;
				CoordType &dim = this->dim;
				easy3d::Vec<3, int> &siz = this->siz;
				CoordType &voxel = this->voxel;

				assert(!_bbox.IsNull());
				bbox=_bbox;
				dim  = bbox.max() - bbox.min();
				assert((grid_size[0]>0)&&(grid_size[1]>0)&&(grid_size[2]>0));
				siz=grid_size;

				voxel[0] = dim[0]/siz[0];
				voxel[1] = dim[1]/siz[1];
				voxel[2] = dim[2]/siz[2];
				hash_table.clear();
		}

		template <class OBJITER>
		void Set(const OBJITER & _oBegin, const OBJITER & _oEnd, const Box3x &_bbox=Box3x() )
		{
			OBJITER i;
			Box3x b;
			Box3x &bbox = this->bbox;
			CoordType &dim = this->dim;
			easy3d::Vec<3, int> &siz = this->siz;
			CoordType &voxel = this->voxel;

			int _size=(int)std::distance<OBJITER>(_oBegin,_oEnd);
			if(!_bbox.IsNull()) this->bbox=_bbox;
			else
			{
					for(i = _oBegin; i!= _oEnd; ++i)
					{
						b.add_point((*i))
						this->bbox.add_box(b);
					}
					bbox.Offset(bbox.diagonal()/100.0) ;
			}

			dim  = bbox.max() - bbox.min();
			BestDim(_size, dim, siz );
			// find voxel size
			voxel[0] = dim[0]/siz[0];
			voxel[1] = dim[1]/siz[1];
			voxel[2] = dim[2]/siz[2];

			for(i = _oBegin; i!= _oEnd; ++i)
				Add(&(*i));
		}


		void GridReal( const easy3d::Vec<3, float> & p, CellIterator & first, CellIterator & last )
		{
			easy3d::Vec<3, int> _c;
			this->PToIP(p,_c);
			Grid(_c,first,last);
		}

		void Grid( int x,int y,int z, CellIterator & first, CellIterator & last )
		{
			this->Grid(easy3d::Vec<3, int>(x,y,z),first,last);
		}

		void Grid( const easy3d::Vec<3, int> & _c, CellIterator & first, CellIterator & end )
		{
			std::pair<HashIterator, HashIterator> CellRange = hash_table.equal_range(_c);
			first.t = CellRange.first;
			end.t = CellRange.second;
		}

		void Clear()
		{
			hash_table.clear();
			AllocatedCells.clear();
		}

		//
		int CountInSphere(const easy3d::vec3 &p, const float radius, std::vector<HashIterator> &inSphVec)
		{
			Box3x b(p - CoordType(radius, radius, radius), p + CoordType(radius, radius, radius));
			easy3d::GenericBox3<int> bb;
			this->BoxToIBox(b, bb);
			float r2 = radius * radius;
			inSphVec.clear();

			for (int i = bb.x_min(); i <= bb.x_max(); i++)
				for (int j = bb.y_min(); j <= bb.y_max(); j++)
					for (int k = bb.z_min(); k <= bb.z_max(); k++)
					{
						std::pair<HashIterator, HashIterator> CellRange = hash_table.equal_range(easy3d::Vec<3, int>(i, j, k));
						for (HashIterator hi = CellRange.first; hi != CellRange.second; ++hi)
						{
							if (easy3d::distance2(p, *(hi->second)) <= r2)
								inSphVec.push_back(hi);
						}
					}
			return int(inSphVec.size());
		}
	}; // end class

	
}// end namespace

#endif