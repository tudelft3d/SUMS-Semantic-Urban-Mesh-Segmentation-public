// Copyright (c) 2017 GeometryFactory
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-4.14/Polyhedron_IO/include/CGAL/IO/PLY_reader.h $
// $Id: PLY_reader.h 1cfcb1b %aI Simon Giraudot
// SPDX-License-Identifier: LGPL-3.0+
//
// Author(s)     : Simon Giraudot

#ifndef CGAL_IO_PLY_READER_H
#define CGAL_IO_PLY_READER_H

#include "CGAL/IO/read_ply_points.h" //#include <CGAL/IO/read_ply_points.h>

namespace CGAL {

	namespace internal
	{
		template <typename Integer, class Polygon_3, class Color_rgb>
		bool
			read_PLY_faces(std::istream& in,
				internal::PLY::PLY_element& element,
				std::vector< Polygon_3 >& polygons,
				std::vector< Color_rgb >& fcolors,
				const char* vertex_indices_tag)
		{
			bool has_colors = false;
			std::string rtag = "r", gtag = "g", btag = "b";
			if (((element.has_property<boost::uint8_t>("red") || element.has_property<boost::uint8_t>("r")) &&
				(element.has_property<boost::uint8_t>("green") || element.has_property<boost::uint8_t>("g")) &&
				(element.has_property<boost::uint8_t>("blue") || element.has_property<boost::uint8_t>("b"))))

			{
				has_colors = true;
				if (element.has_property<boost::uint8_t>("red") )
				{
					rtag = "red"; gtag = "green"; btag = "blue";
				}
			}

			for (std::size_t j = 0; j < element.number_of_items(); ++j)
			{
				for (std::size_t k = 0; k < element.number_of_properties(); ++k)
				{
					internal::PLY::PLY_read_number* property = element.property(k);
					property->get(in);

					if (in.fail())
						return false;
				}

				cpp11::tuple<std::vector<Integer>, boost::uint8_t, boost::uint8_t, boost::uint8_t> new_face;

				if (has_colors)
				{
					PLY::process_properties(element, new_face,
						std::make_pair(CGAL::make_nth_of_tuple_property_map<0>(new_face),
							PLY_property<std::vector<Integer> >(vertex_indices_tag)),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(new_face),
							PLY_property<boost::uint8_t>(rtag.c_str())),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(new_face),
							PLY_property<boost::uint8_t>(gtag.c_str())),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(new_face),
							PLY_property<boost::uint8_t>(btag.c_str())));

					fcolors.push_back(Color_rgb(get<1>(new_face), get<2>(new_face), get<3>(new_face)));
				}
				else
					PLY::process_properties(element, new_face,
						std::make_pair(CGAL::make_nth_of_tuple_property_map<0>(new_face),
							PLY_property<std::vector<Integer> >(vertex_indices_tag)));

				polygons.push_back(Polygon_3(get<0>(new_face).size()));
				for (std::size_t i = 0; i < get<0>(new_face).size(); ++i)
					polygons.back()[i] = std::size_t(get<0>(new_face)[i]);
			}

			return true;
		}



		//***********************Weixiao Update*******************************//
		template <typename Integer, class Polygon_3, class Color_rgb>
		bool
			read_PLY_faces(std::istream& in,
				internal::PLY::PLY_element& element,
				std::vector< Polygon_3 >& polygons,
				std::vector< Color_rgb >& fcolors,
				std::vector< int >& flabels,
				std::vector<std::vector<float>>& fi_texcoord,
				std::vector<int>& texture_id,
				std::vector<float>& fi_prob,
				std::vector<int>& fi_segment_id,
				const char* vertex_indices_tag)
		{
			bool has_colors = false;
			std::string rtag = "r", gtag = "g", btag = "b";

			std::cerr << element.has_property<float>("r") << ' ' <<
				element.has_property<float>("g") << ' ' <<
				element.has_property<float>("b") << ' ' << std::endl;

			if (((element.has_property<boost::uint8_t>("red") || element.has_property<boost::uint8_t>("r")) &&
				(element.has_property<boost::uint8_t>("green") || element.has_property<boost::uint8_t>("g")) &&
				(element.has_property<boost::uint8_t>("blue") || element.has_property<boost::uint8_t>("b"))) ||
				/**/
				((element.has_property<float>("red") || element.has_property<float>("r")) &&
				(element.has_property<float>("green") || element.has_property<float>("g")) &&
					(element.has_property<float>("blue") || element.has_property<float>("b"))))
			{
				has_colors = true;
				/**/
				if (element.has_property<boost::uint8_t>("red") || element.has_property<float>("red"))
				{
					rtag = "red"; gtag = "green"; btag = "blue";
				}
			}

			std::vector<float> color;
			std::vector<float> texcoord_temp;
			for (std::size_t j = 0; j < element.number_of_items(); ++j)
			{
				for (std::size_t k = 0; k < element.number_of_properties(); ++k)
				{
					internal::PLY::PLY_read_number* property = element.property(k);

					if (element.property(k)->name() == "label")
					{
						flabels.push_back(property->read<int>(in));
					}
					else if (element.property(k)->name() == "texCoord" || element.property(k)->name() == "texcoord")
					{
						property->read<float>(in);
						texcoord_temp.push_back(property->read<float>(in));
						texcoord_temp.push_back(property->read<float>(in));
						texcoord_temp.push_back(property->read<float>(in));
						texcoord_temp.push_back(property->read<float>(in));
						texcoord_temp.push_back(property->read<float>(in));
						texcoord_temp.push_back(property->read<float>(in));
					}
					else if (element.property(k)->name() == "texnumber")
					{
						texture_id.push_back(property->read<int>(in));
					}
					else if (element.property(k)->name() == "label_probabilities")
					{
						fi_prob.push_back(property->read<float>(in));
					}
					else if (element.property(k)->name() == "face_segment_id")
					{
						fi_segment_id.push_back(property->read<float>(in));
					}
					/**/
					else if (has_colors && (element.has_property<float>("red") || element.has_property<float>("r")) && element.property(k)->name() == rtag.c_str())
					{
						color.push_back(property->read<float>(in));
					}
					else if (has_colors && (element.has_property<float>("green") || element.has_property<float>("g")) && element.property(k)->name() == gtag.c_str())
					{
						color.push_back(property->read<float>(in));
					}
					else if (has_colors && (element.has_property<float>("blue") || element.has_property<float>("b")) && element.property(k)->name() == btag.c_str())
					{
						color.push_back(property->read<float>(in));
					}
					else
					{
						property->get(in);
					}

					if (in.fail())
						return false;
				}

				if (color.size() == 3) {
					fcolors.push_back(Color_rgb(color[0] * 255, color[1] * 255, color[2] * 255));
				}
				else {
					std::cerr << "color size != 3 , got" << color.size() << std::endl;
				}
				color.clear();

				if (texcoord_temp.empty() == false && texcoord_temp.size() == 6)
				{
					fi_texcoord.push_back(texcoord_temp);
					texcoord_temp.clear();
				}

				cpp11::tuple<std::vector<Integer>, boost::uint8_t, boost::uint8_t, boost::uint8_t> new_face;
				//std::cerr << has_colors << std::endl;
				if (has_colors && !(element.has_property<float>("red") || element.has_property<float>("r")))
				{
					PLY::process_properties(element, new_face,
						std::make_pair(CGAL::make_nth_of_tuple_property_map<0>(new_face),
							PLY_property<std::vector<Integer> >(vertex_indices_tag)),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(new_face),
							PLY_property<boost::uint8_t>(rtag.c_str())),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(new_face),
							PLY_property<boost::uint8_t>(gtag.c_str())),
						std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(new_face),
							PLY_property<boost::uint8_t>(btag.c_str())));

					fcolors.push_back(Color_rgb(get<1>(new_face), get<2>(new_face), get<3>(new_face)));
				}
				else
					PLY::process_properties(element, new_face,
						std::make_pair(CGAL::make_nth_of_tuple_property_map<0>(new_face),
							PLY_property<std::vector<Integer> >(vertex_indices_tag)));

				polygons.push_back(Polygon_3(get<0>(new_face).size()));
				for (std::size_t i = 0; i < get<0>(new_face).size(); ++i)
					polygons.back()[i] = std::size_t(get<0>(new_face)[i]);
			}
			return true;
		}
	}
	//*******************************************************************//

	template <class Point_3, class Polygon_3>
	bool
		read_PLY(std::istream& in,
			std::vector< Point_3 >& points,
			std::vector< Polygon_3 >& polygons,
			bool /* verbose */ = false)
	{
		if (!in)
		{
			std::cerr << "Error: cannot open file" << std::endl;
			return false;
		}

		internal::PLY::PLY_reader reader;

		if (!(reader.init(in)))
		{
			in.setstate(std::ios::failbit);
			return false;
		}

		for (std::size_t i = 0; i < reader.number_of_elements(); ++i)
		{
			internal::PLY::PLY_element& element = reader.element(i);

			if (element.name() == "vertex" || element.name() == "vertices")
			{
				for (std::size_t j = 0; j < element.number_of_items(); ++j)
				{
					for (std::size_t k = 0; k < element.number_of_properties(); ++k)
					{
						internal::PLY::PLY_read_number* property = element.property(k);
						property->get(in);

						if (in.fail())
							return false;
					}

					Point_3 new_vertex;

					internal::PLY::process_properties(element, new_vertex,
						make_ply_point_reader(CGAL::Identity_property_map<Point_3>()));

					points.push_back(get<0>(new_vertex));
				}
			}
			else if (element.name() == "face" || element.name() == "faces")
			{
				std::vector<CGAL::Color> dummy;

				if (element.has_property<std::vector<boost::int32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, dummy, "vertex_indices");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, dummy, "vertex_indices");
				else if (element.has_property<std::vector<boost::int32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, dummy, "vertex_index");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, dummy, "vertex_index");
				else
				{
					std::cerr << "Error: can't find vertex indices in PLY input" << std::endl;
					return false;
				}
			}
			else // Read other elements and ignore
			{
				for (std::size_t j = 0; j < element.number_of_items(); ++j)
				{
					for (std::size_t k = 0; k < element.number_of_properties(); ++k)
					{
						internal::PLY::PLY_read_number* property = element.property(k);
						property->get(in);

						if (in.fail())
							return false;
					}
				}
			}
		}

		return true;
	}

	template <class Point_3, class Polygon_3, class Color_rgb>
	bool
		read_PLY(std::istream& in,
			std::vector< Point_3 >& points,
			std::vector< Polygon_3 >& polygons,
			std::vector<Color_rgb>& fcolors,
			std::vector<Color_rgb>& vcolors,
			bool /* verbose */ = false)
	{
		if (!in)
		{
			std::cerr << "Error: cannot open file" << std::endl;
			return false;
		}
		internal::PLY::PLY_reader reader;

		if (!(reader.init(in)))
		{
			in.setstate(std::ios::failbit);
			return false;
		}

		for (std::size_t i = 0; i < reader.number_of_elements(); ++i)
		{
			internal::PLY::PLY_element& element = reader.element(i);

			if (element.name() == "vertex" || element.name() == "vertices")
			{
				bool has_colors = false;
				std::string rtag = "r", gtag = "g", btag = "b";
				if ((element.has_property<boost::uint8_t>("red") || element.has_property<boost::uint8_t>("r")) &&
					(element.has_property<boost::uint8_t>("green") || element.has_property<boost::uint8_t>("g")) &&
					(element.has_property<boost::uint8_t>("blue") || element.has_property<boost::uint8_t>("b")))
				{
					has_colors = true;
					if (element.has_property<boost::uint8_t>("red"))
					{
						rtag = "red"; gtag = "green"; btag = "blue";
					}
				}

				for (std::size_t j = 0; j < element.number_of_items(); ++j)
				{
					for (std::size_t k = 0; k < element.number_of_properties(); ++k)
					{
						internal::PLY::PLY_read_number* property = element.property(k);
						property->get(in);

						if (in.fail())
							return false;
					}

					cpp11::tuple<Point_3, boost::uint8_t, boost::uint8_t, boost::uint8_t> new_vertex;

					if (has_colors)
					{
						internal::PLY::process_properties(element, new_vertex,
							make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(new_vertex)),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(new_vertex),
								PLY_property<boost::uint8_t>(rtag.c_str())),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(new_vertex),
								PLY_property<boost::uint8_t>(gtag.c_str())),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(new_vertex),
								PLY_property<boost::uint8_t>(btag.c_str())));

						vcolors.push_back(Color_rgb(get<1>(new_vertex), get<2>(new_vertex), get<3>(new_vertex)));
					}
					else
						internal::PLY::process_properties(element, new_vertex,
							make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(new_vertex)));

					points.push_back(get<0>(new_vertex));
				}
			}
			else if (element.name() == "face" || element.name() == "faces")
			{
				if (element.has_property<std::vector<boost::int32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, fcolors, "vertex_indices");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, fcolors, "vertex_indices");
				else if (element.has_property<std::vector<boost::int32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, fcolors, "vertex_index");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, fcolors, "vertex_index");
				else
				{
					std::cerr << "Error: can't find vertex indices in PLY input" << std::endl;
					return false;
				}
			}
			else // Read other elements and ignore
			{
				for (std::size_t j = 0; j < element.number_of_items(); ++j)
				{
					for (std::size_t k = 0; k < element.number_of_properties(); ++k)
					{
						internal::PLY::PLY_read_number* property = element.property(k);
						property->get(in);

						if (in.fail())
							return false;
					}
				}
			}
		}

		return true;
	}


	//***********************Weixiao Update*******************************//
	template <class Point_3, class Polygon_3, class Color_rgb>
	bool
		read_PLY(std::istream& in,
			std::vector< Point_3 >& points,
			std::vector< Polygon_3 >& polygons,
			std::vector<Color_rgb>& fcolors,
			std::vector<Color_rgb>& vcolors,
			std::vector< int >& flabels,
			std::vector<std::string>& flabelcomments,
			std::vector<std::vector<float>>& fi_texcoord,
			std::vector<int>& texture_id,
			std::vector<std::string>& texture_name,
			std::vector<float>& fi_prob,
			std::vector<int>& fi_segment_id,
			bool /* verbose */ = false)
	{
		if (!in)
		{
			std::cerr << "Error: cannot open file" << std::endl;
			return false;
		}
		internal::PLY::PLY_reader reader;

		if (!(reader.init(in)))
		{
			in.setstate(std::ios::failbit);
			return false;
		}

		std::string comments = reader.comments();
		std::istringstream iss(comments);
		std::string line, word;
		while (getline(iss, line))
		{
			if (line != "Generated by the CGAL library") // Avoid repeating the line if multiple savings
			{
				std::istringstream temp_stream(line);
				int space_count = 0;
				bool is_label = false, is_texture_name = false;
				while (temp_stream >> word)
				{
					if (word == "label" && is_label == false)
						is_label = true;

					++space_count;
					if (space_count == 3 && is_label == true)
					{
						//Notice the class name must be one word or words connect with underscore "_"
						flabelcomments.push_back(word);
					}

					if (word == "TextureFile" && is_texture_name == false)
					{
						is_texture_name = true;
					}

					if (is_label == false && is_texture_name == true && word != "TextureFile")
					{
						texture_name.push_back(word);
					}
				}
			}
		}

		for (std::size_t i = 0; i < reader.number_of_elements(); ++i)
		{
			internal::PLY::PLY_element& element = reader.element(i);

			if (element.name() == "vertex" || element.name() == "vertices")
			{
				bool has_colors = false;
				std::string rtag = "r", gtag = "g", btag = "b";
				if (((element.has_property<boost::uint8_t>("red") || element.has_property<boost::uint8_t>("r")) &&
					(element.has_property<boost::uint8_t>("green") || element.has_property<boost::uint8_t>("g")) &&
					(element.has_property<boost::uint8_t>("blue") || element.has_property<boost::uint8_t>("b"))) )

				{
					has_colors = true;

					if (element.has_property<boost::uint8_t>("red") )
					{
						rtag = "red"; gtag = "green"; btag = "blue";
					}
				}

				for (std::size_t j = 0; j < element.number_of_items(); ++j)
				{
					for (std::size_t k = 0; k < element.number_of_properties(); ++k)
					{
						internal::PLY::PLY_read_number* property = element.property(k);
						property->get(in);

						if (in.fail())
							return false;
					}

					cpp11::tuple<Point_3, boost::uint8_t, boost::uint8_t, boost::uint8_t> new_vertex;

					if (has_colors)
					{
						internal::PLY::process_properties(element, new_vertex,
							make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(new_vertex)),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(new_vertex),
								PLY_property<boost::uint8_t>(rtag.c_str())),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(new_vertex),
								PLY_property<boost::uint8_t>(gtag.c_str())),
							std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(new_vertex),
								PLY_property<boost::uint8_t>(btag.c_str())));

						vcolors.push_back(Color_rgb(get<1>(new_vertex), get<2>(new_vertex), get<3>(new_vertex)));
					}
					else
						internal::PLY::process_properties(element, new_vertex,
							make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(new_vertex)));

					points.push_back(get<0>(new_vertex));
				}
			}

			if (element.name() == "face" || element.name() == "faces")
			{
				if (element.has_property<std::vector<boost::int32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, fcolors, flabels, fi_texcoord, texture_id, fi_prob, fi_segment_id, "vertex_indices");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_indices"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, fcolors, flabels, fi_texcoord, texture_id, fi_prob, fi_segment_id, "vertex_indices");
				else if (element.has_property<std::vector<boost::int32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::int32_t>(in, element, polygons, fcolors, flabels, fi_texcoord, texture_id, fi_prob, fi_segment_id, "vertex_index");
				else if (element.has_property<std::vector<boost::uint32_t> >("vertex_index"))
					internal::read_PLY_faces<boost::uint32_t>(in, element, polygons, fcolors, flabels, fi_texcoord, texture_id, fi_prob, fi_segment_id, "vertex_index");
				else
				{
					std::cerr << "Error: can't find vertex indices in PLY input" << std::endl;
					return false;
				}
			}
		}

		return true;
	}

	//*******************************************************************//

} // namespace CGAL

#endif // CGAL_IO_PLY_READER_H