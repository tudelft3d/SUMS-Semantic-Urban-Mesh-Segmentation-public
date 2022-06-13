#include <iostream>
#include <cstdio>
#include <vector>
#include <boost/python.hpp>
#include <boost/numpy.hpp>
#include <numpy/ndarrayobject.h>
#include "boost/tuple/tuple.hpp"
#include "boost/python/object.hpp"

#include <easy3d/surface_mesh.h>
#include <easy3d/point_cloud.h>
#include <easy3d/kdtree.h>
#include <easy3d/point_cloud_io.h>
#include <easy3d/surface_mesh_io.h>

namespace bpn = boost::numpy;
namespace bp =  boost::python;

//typedef boost::tuple< std::vector< std::vector<uint32_t> >, std::vector<uint32_t> > Custom_tuple;
typedef boost::tuple< std::vector< std::vector<uint32_t> >, std::vector<uint32_t>, std::vector< std::vector< float > >, std::vector< std::vector< float > >> Custom_tuple;
typedef boost::tuple<std::vector< std::vector< float > >, std::vector<uint32_t>> Aug_tuple;

template<class T>
int arrayVlenth(T *p)
{
	int len = 0;
	while (*p)
	{
		p++;
		len++;
	}
	return len;
}

uint32_t arrayVlenth2(uint32_t *p)
{
	int len = 0;
	while (*p)
	{
		p++;
		len++;
	}
	return len;
}

struct VecToArray
{//converts a vector<uint32_t> to a numpy array
    static PyObject * convert(const std::vector<uint32_t> & vec)
    {
        npy_intp dims = vec.size();
        PyObject * obj = PyArray_SimpleNew(1, &dims, NPY_UINT32);
        void * arr_data = PyArray_DATA((PyArrayObject*)obj);
        memcpy(arr_data, &vec[0], dims * sizeof(uint32_t));
        return obj;
    }
};

struct VecToArray_float
{//converts a vector<uint32_t> to a numpy array
    static PyObject * convert(const std::vector<float> & vec)
    {
        npy_intp dims = vec.size();
        PyObject * obj = PyArray_SimpleNew(1, &dims, NPY_FLOAT32);
        void * arr_data = PyArray_DATA((PyArrayObject*)obj);
        memcpy(arr_data, &vec[0], dims * sizeof(float));
        return obj;
    }
};


template<class T>
struct VecvecToList
{//converts a vector< vector<T> > to a list
        static PyObject* convert(const std::vector< std::vector<T> > & vecvec)
    {
        boost::python::list* pylistlist = new boost::python::list();
        for(size_t i = 0; i < vecvec.size(); i++)
        {
            boost::python::list* pylist = new boost::python::list();
            for(size_t j = 0; j < vecvec[i].size(); j++)
            {
                pylist->append(vecvec[i][j]);
            }
            pylistlist->append((pylist, pylist[0]));
        }
        return pylistlist->ptr();
    }
};

template <class T>
struct VecvecToArray
{//converts a vector< vector<uint32_t> > to a numpy 2d array
	static PyObject * convert(const std::vector< std::vector<T> > & vecvec)
	{
		npy_intp dims[2];
		dims[0] = vecvec.size();
		dims[1] = vecvec[0].size();
		PyObject * obj;
		if (typeid(T) == typeid(uint8_t))
			obj = PyArray_SimpleNew(2, dims, NPY_UINT8);
		else if (typeid(T) == typeid(float))
			obj = PyArray_SimpleNew(2, dims, NPY_FLOAT32);
		else if (typeid(T) == typeid(uint32_t))
			obj = PyArray_SimpleNew(2, dims, NPY_UINT32);
		void * arr_data = PyArray_DATA((PyArrayObject*)obj);
		std::size_t cell_size = sizeof(T);
		for (std::size_t i = 0; i < dims[0]; i++)
		{
			memcpy((char *)arr_data + i * dims[1] * cell_size, &(vecvec[i][0]), dims[1] * cell_size);
		}
		return obj;
	}
};

struct to_py_tuple
{//converts output to a python tuple
	static PyObject* convert(const Custom_tuple& c_tuple) {
		bp::list values;
		//add all c_tuple items to "values" list

		PyObject * vecvec_pyo = VecvecToList<uint32_t>::convert(c_tuple.get<0>());
		PyObject * vec_pyo = VecToArray::convert(c_tuple.get<1>());
		PyObject * ptsf_vec_pyo = VecvecToArray<float>::convert(c_tuple.get<2>());
		PyObject * segf_vec_pyo = VecvecToArray<float>::convert(c_tuple.get<3>());

		values.append(bp::handle<>(bp::borrowed(vecvec_pyo)));
		values.append(bp::handle<>(bp::borrowed(vec_pyo)));
		values.append(bp::handle<>(bp::borrowed(ptsf_vec_pyo)));
		values.append(bp::handle<>(bp::borrowed(segf_vec_pyo)));

		return bp::incref(bp::tuple(values).ptr());
	}
};

struct to_py_aug_tuple
{//converts output to a python tuple
	static PyObject* convert(const Aug_tuple& c_tuple) {
		bp::list values;
		//add all c_tuple items to "values" list
		PyObject * feas_pyo = VecvecToArray<float>::convert(c_tuple.get<0>());
		PyObject * labels_pyo = VecToArray::convert(c_tuple.get<1>());

		values.append(bp::handle<>(bp::borrowed(feas_pyo)));
		values.append(bp::handle<>(bp::borrowed(labels_pyo)));
		return bp::incref(bp::tuple(values).ptr());
	}
};

//************************************************************************************************************************************//
	
//--- get feature properties from point cloud ---
void get_segment_properties
(
	easy3d::PointCloud *pcl,
	easy3d::PointCloud::VertexProperty < std::vector<int> > &feas,
	std::string &v_fea
)
{
	//j = features
	feas = pcl->get_vertex_property<std::vector<int>>(v_fea);
}

void get_segment_properties
(
	easy3d::PointCloud *pcl,
	easy3d::PointCloud::VertexProperty < std::vector<float> > &feas,
	std::string &v_fea
)
{
	//j = features
	feas = pcl->get_vertex_property<std::vector<float>>(v_fea);
}

//read features point cloud
void get_all_feature_properties_from_feature_point_cloud
(
	easy3d::PointCloud *pcl,
	std::vector<std::vector<uint32_t>> &seg_face_vec,
	std::vector<int> &seg_truth,
	std::vector< std::vector<float>> & basic_feas,
	std::vector< std::vector<float> > &segment_eigen_feas,
	std::vector< std::vector<float> > &segment_color_feas,
	std::vector< std::vector<float> > &mulsc_ele_feas,
	const int labels_name_size
)
{
	std::cout << "	Get feature point cloud cost ";
	const double t_total = omp_get_wtime();
	easy3d::PointCloud::VertexProperty < std::vector<int> > p_faces;
	easy3d::PointCloud::VertexProperty < std::vector<float> > p_shp_feas, p_mulsc_ele_feas;
	easy3d::PointCloud::VertexProperty < std::vector<float> > p_segment_eigen_feas, p_segment_color_feas;
	std::map<int, bool> use_feas = { {0, false}, {1, false}, {2, false}, {3, false} };

	//feature names in files
	std::vector<std::pair<std::string, int>> selected_pcl_vertex_features
	{
		{"v:segment_basic_features", 0},
		{"v:segment_eigen_features", 1},
		{"v:segment_color_features", 2},
		{"v:multiscale_elevation_features", 3}
	};

	get_segment_properties(pcl, p_faces, std::string("v:mesh_faces_id"));
	for (auto sf : selected_pcl_vertex_features)
	{
		switch (sf.second)
		{
		case 0: get_segment_properties(pcl, p_shp_feas, sf.first); use_feas[0] = true; break;
		case 1: get_segment_properties(pcl, p_segment_eigen_feas, sf.first); use_feas[1] = true; break;
		case 2: get_segment_properties(pcl, p_segment_color_feas, sf.first); use_feas[2] = true; break;
		case 3: get_segment_properties(pcl, p_mulsc_ele_feas, sf.first); use_feas[3] = true; break;
		}
	}

	easy3d::PointCloud::Vertex p_ini(0);//feature size
	seg_face_vec = std::vector< std::vector<uint32_t>>(pcl->n_vertices(), std::vector<uint32_t>());
	basic_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());
	mulsc_ele_feas = std::vector< std::vector<float>>(pcl->n_vertices(), std::vector<float>());
	//i,j,k = segments, scales, features(gradient)
	segment_eigen_feas = std::vector< std::vector<float> >(pcl->n_vertices(), std::vector<float>());
	segment_color_feas = std::vector< std::vector<float> >(pcl->n_vertices(), std::vector<float>());

	for (int sfi = 0; sfi < pcl->vertices_size(); ++sfi)
	{
		easy3d::PointCloud::Vertex ptx(sfi);
		seg_truth.push_back(pcl->get_vertex_property<int>("v:label")[ptx]);
		seg_face_vec[sfi].insert(seg_face_vec[sfi].end(), p_faces[ptx].begin(), p_faces[ptx].end());
		if (use_feas[0])
			basic_feas[sfi].insert(basic_feas[sfi].end(), p_shp_feas[ptx].begin(), p_shp_feas[ptx].end());
		if (use_feas[3])
			mulsc_ele_feas[sfi].insert(mulsc_ele_feas[sfi].end(), p_mulsc_ele_feas[ptx].begin(), p_mulsc_ele_feas[ptx].end());

		if (use_feas[1])
			segment_eigen_feas[sfi].insert(segment_eigen_feas[sfi].end(), p_segment_eigen_feas[ptx].begin(), p_segment_eigen_feas[ptx].end());
		if (use_feas[2])
			segment_color_feas[sfi].insert(segment_color_feas[sfi].end(), p_segment_color_feas[ptx].begin(), p_segment_color_feas[ptx].end());
	}

	std::cout << omp_get_wtime() - t_total << " (s)" << std::endl;
}

void parsing_from_mulsfeas_to_feas
(
	easy3d::PointCloud *pcl,
	std::vector<int> &seg_truth,
	std::vector< std::vector<float>> & basic_feas,
	std::vector< std::vector<float> > &segment_eigen_feas,
	std::vector< std::vector<float> > &segment_color_feas,
	std::vector< std::vector<float> > &mulsc_ele_feas,
	const int labels_name_size,
	std::vector<std::vector<float>> &feas_out,
	std::vector<uint32_t> &labels_out
)
{
	std::cout << "	Expand the feature vector cost ";
	const double t_total = omp_get_wtime();

	feas_out.resize(pcl->vertices_size());
	auto get_point_coord = pcl->get_vertex_property<easy3d::vec3>("v:point");
	for (int pi = 0; pi < pcl->vertices_size(); ++pi)
	{
		easy3d::PointCloud::Vertex ptx(pi);
		labels_out.push_back(seg_truth[pi]);

		feas_out[pi].push_back(get_point_coord[ptx].x);
		feas_out[pi].push_back(get_point_coord[ptx].y);
		feas_out[pi].push_back(get_point_coord[ptx].z);

		feas_out[pi].insert(feas_out[pi].end(), basic_feas[pi].begin(), basic_feas[pi].end());
		feas_out[pi].insert(feas_out[pi].end(), mulsc_ele_feas[pi].begin(), mulsc_ele_feas[pi].end());

		feas_out[pi].insert(feas_out[pi].end(), segment_eigen_feas[pi].begin(), segment_eigen_feas[pi].end());
		feas_out[pi].insert(feas_out[pi].end(), segment_color_feas[pi].begin(), segment_color_feas[pi].end());
	}

	std::cout << omp_get_wtime() - t_total << " (s)" << std::endl;
}

//read point cloud data and parsing to python array
PyObject *read_data_for_augmentation
(
	const bp::str read_data_path_py,
	const bp::str labels_name_size_bpstr = "6"
)
{
	//read *.ply
	std::ostringstream pcl_str_ostemp;
	std::string read_data_path = bp::extract<char const *>(read_data_path_py);
	std::string labels_name_size_str = bp::extract<char const *>(labels_name_size_bpstr);
	int labels_name_size = std::stoi(labels_name_size_str);

	std::cout << "	Start to read data for parsing to python " << read_data_path << std::endl;

	pcl_str_ostemp << read_data_path;
	std::string pcl_str_temp = pcl_str_ostemp.str().data();
	char * pclPath_temp = (char *)pcl_str_temp.data();
	easy3d::PointCloud* pcl_in = easy3d::PointCloudIO::load(pclPath_temp);

	//features in
	std::vector<int> seg_truth;
	std::vector< std::vector<uint32_t>> seg_face_vec;
	std::vector< std::vector<float> > basic_feas, mulsc_ele_feas;
	std::vector< std::vector<float> > segment_eigen_feas, segment_color_feas;

	get_all_feature_properties_from_feature_point_cloud
	(
		pcl_in, seg_face_vec, seg_truth,
		basic_feas, segment_eigen_feas, segment_color_feas,
		mulsc_ele_feas, 
		labels_name_size
	);

	//features out
	std::vector<std::vector<float>> feas_out;
	std::vector<uint32_t> labels_out;
	parsing_from_mulsfeas_to_feas
	(
		pcl_in, seg_truth,
		basic_feas, segment_eigen_feas, segment_color_feas, 
		mulsc_ele_feas, 
		labels_name_size,
		feas_out, labels_out
	);

	delete pcl_in;
	return to_py_aug_tuple::convert(Aug_tuple(feas_out, labels_out));
}

void update_all_feature_properties_for_feature_point_cloud
(
	easy3d::PointCloud* pcl,
	const bpn::ndarray feas_aug,
	const bpn::ndarray labels_aug,
	const int labels_name_size
)
{
	float *feas_aug_data = reinterpret_cast<float*>(feas_aug.get_data());
	int *labels_aug_data = reinterpret_cast<int*>(labels_aug.get_data());

	const uint32_t n_pts = bp::len(feas_aug);

	//feature names in files
	std::vector<std::pair<std::string, int>> selected_pcl_vertex_features
	{
		{"v:segment_basic_features", 0},
		{"v:segment_eigen_features", 1},
		{"v:segment_color_features", 2},
		{"v:multiscale_elevation_features", 3}
	};

	easy3d::PointCloud::VertexProperty < std::vector<int> > p_faces;
	easy3d::PointCloud::VertexProperty < std::vector<float> > p_shp_feas, p_mulsc_ele_feas;
	easy3d::PointCloud::VertexProperty<std::vector<float>> p_segment_eigen_feas, p_segment_color_feas;
	std::map<int, bool> use_feas = { {0, false}, {1, false}, {2, false}, {3, false} };

	//get_segment_properties(pcl_out, p_faces, std::string("v:mesh_faces_id"));
	for (auto sf : selected_pcl_vertex_features)
	{
		switch (sf.second)
		{
		case 0: get_segment_properties(pcl, p_shp_feas, sf.first); use_feas[0] = true; break;
		case 1: get_segment_properties(pcl, p_segment_eigen_feas, sf.first); use_feas[1] = true; break;
		case 2: get_segment_properties(pcl, p_segment_color_feas, sf.first); use_feas[2] = true; break;
		case 3: get_segment_properties(pcl, p_mulsc_ele_feas, sf.first); use_feas[3] = true; break;;
		}
	}

	//std::cout << " feas_aug_length = " << arrayVlenth(feas_aug_data) << std::endl;
	easy3d::PointCloud::Vertex vt0(0);
	int fi = 0;
	for (int i = 0; i < n_pts; ++i)
	{
		pcl->add_vertex(easy3d::vec3(feas_aug_data[fi], feas_aug_data[fi + 1], feas_aug_data[fi + 2]));
		fi += 3;
		pcl->get_vertex_property<int>("v:label")[*(--pcl->vertices_end())] = labels_aug_data[i];

		p_shp_feas[*(--pcl->vertices_end())].resize(p_shp_feas[vt0].size());
		for (int j = 0; j < p_shp_feas[vt0].size(); ++j)
		{
			p_shp_feas[*(--pcl->vertices_end())][j] = feas_aug_data[fi];
			fi += 1;
		}

		p_mulsc_ele_feas[*(--pcl->vertices_end())].resize(p_mulsc_ele_feas[vt0].size());
		for (int j = 0; j < p_mulsc_ele_feas[vt0].size(); ++j)
		{
			p_mulsc_ele_feas[*(--pcl->vertices_end())][j] = feas_aug_data[fi];
			fi += 1;
		}

		p_segment_eigen_feas[*(--pcl->vertices_end())].resize(p_segment_eigen_feas[vt0].size());
		for (int j = 0; j < p_segment_eigen_feas[vt0].size(); ++j)
		{
			p_segment_eigen_feas[*(--pcl->vertices_end())][j] = feas_aug_data[fi];
			fi += 1;
		}

		p_segment_color_feas[*(--pcl->vertices_end())].resize(p_segment_color_feas[vt0].size());
		for (int j = 0; j < p_segment_color_feas[vt0].size(); ++j)
		{
			p_segment_color_feas[*(--pcl->vertices_end())][j] = feas_aug_data[fi];
			fi += 1;
		}
	}

	pcl->delete_vertex(vt0);
	pcl->garbage_collection();
}

//read python array and parsing to point cloud data 
void write_data_for_augmentation
(
	const bp::str read_data_path_py,
	const bp::str write_data_path_py,
	const bpn::ndarray feas_aug,
	const bpn::ndarray labels_aug,
	const bp::str labels_name_size_bpstr = "6"
)
{
	//read *.ply
	std::ostringstream pcl_str_ostemp_in;
	std::string read_data_path = bp::extract<char const *>(read_data_path_py);
	pcl_str_ostemp_in << read_data_path;
	std::string pcl_str_temp_in = pcl_str_ostemp_in.str().data();
	char * pclPath_temp_in = (char *)pcl_str_temp_in.data();
	easy3d::PointCloud* pcl_out = easy3d::PointCloudIO::load(pclPath_temp_in);
	int old_vcount = pcl_out->vertices_size();
	std::cout << "	Augmentation increased " << bp::len(feas_aug) - old_vcount <<" points." << std::endl;
	pcl_out->resize(1);

	//create output *.ply
	std::ostringstream pcl_str_ostemp_out;
	std::string write_data_path = bp::extract<char const *>(write_data_path_py);
	pcl_str_ostemp_out << write_data_path;
	std::string pcl_str_temp_out = pcl_str_ostemp_out.str().data();
	char * pclPath_temp_out = (char *)pcl_str_temp_out.data();

	std::string labels_name_size_str = bp::extract<char const *>(labels_name_size_bpstr);
	int labels_name_size = std::stoi(labels_name_size_str);

	update_all_feature_properties_for_feature_point_cloud
	(
		pcl_out,
		feas_aug,
		labels_aug,
		labels_name_size
	);

	std::cout << "	Write augmented data ... " << std::endl;
	easy3d::PointCloudIO::save(pclPath_temp_out, pcl_out, true);
	delete pcl_out;
}

BOOST_PYTHON_MODULE(libpp)
{
    _import_array();
    Py_Initialize();
    bpn::initialize();
    bp::to_python_converter< Custom_tuple, to_py_tuple>();
    
	def("read_data_for_augmentation", read_data_for_augmentation);
	def("write_data_for_augmentation", write_data_for_augmentation);
}
