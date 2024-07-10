#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_

#include "../types.hpp"
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Fuzzy_sphere.h>

namespace globe {

using KDTreeTraits = CGAL::Search_traits_3<Kernel>;
using KDTree = CGAL::Kd_tree<KDTreeTraits>;
using FuzzySphere = CGAL::Fuzzy_sphere<KDTreeTraits>;

using Traits = CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel>;
using Triangulation = CGAL::Delaunay_triangulation_on_sphere_2<Traits>;
using FiniteVerticesIterator = Triangulation::Finite_vertices_iterator;
using VertexHandle = Triangulation::Vertex_handle;
using FaceCirculator = Triangulation::Face_circulator;
using FiniteFacesIterator = Triangulation::Finite_faces_iterator;
using FaceHandle = Triangulation::Face_handle;
using EdgeCirculator = Triangulation::Edge_circulator;
using FiniteEdgesIterator = Triangulation::Finite_edges_iterator;
using Edge = Triangulation::Edge;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_
