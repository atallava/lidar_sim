#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

namespace lidar_sim {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel_cgal;
    typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel_cgal> Vb_cgal;
    typedef CGAL::Triangulation_data_structure_2<Vb_cgal>                       Tds_cgal;
    typedef Kernel_cgal::Point_2                                                Point_cgal;
    typedef CGAL::Delaunay_triangulation_2<Kernel_cgal, Tds_cgal> Delaunay_cgal;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel_exact_cgal;
    typedef CGAL::Ray_3<Kernel_exact_cgal> Ray_3_cgal;
    typedef CGAL::Point_3<Kernel_exact_cgal> Point_3_cgal;
    typedef CGAL::Direction_3<Kernel_exact_cgal> Direction_3_cgal;
    typedef Kernel_exact_cgal::Triangle_3 Triangle_3_cgal;

}
