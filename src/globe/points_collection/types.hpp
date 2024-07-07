#ifndef GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_
#define GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_

#include "../types.hpp"
#include <CGAL/Search_traits_3.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Fuzzy_sphere.h>

namespace globe {

typedef CGAL::Search_traits_3<Kernel> KDTreeTraits;
typedef CGAL::Kd_tree<KDTreeTraits> KDTree;
typedef CGAL::Fuzzy_sphere<KDTreeTraits> FuzzySphere;

typedef CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel, SphericalKernel> Traits;
typedef CGAL::Delaunay_triangulation_on_sphere_2<Traits> Triangulation;
typedef Triangulation::Finite_vertices_iterator FiniteVerticesIterator;
typedef Triangulation::Vertex_handle VertexHandle;
typedef Triangulation::Finite_faces_iterator FiniteFacesIterator;
typedef Triangulation::Face_handle FaceHandle;
typedef Triangulation::Edge_circulator EdgeCirculator;
typedef Triangulation::Finite_edges_iterator FiniteEdgesIterator;
typedef Triangulation::Edge Edge;

} // namespace globe

#endif //GLOBEART_SRC_GLOBE_POINTS_COLLECTION_TYPES_H_
