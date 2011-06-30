#ifndef __SIGNAL_PATH_SIMULATOR__
#define __SIGNAL_PATH_SIMULATOR__

#include <cassert>
#include <string>
#include <map>
#include <vector>
#include <list>
#include <boost/tuple/tuple.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>

class SignalPathSimulator
{
 public:
  SignalPathSimulator();
  ~SignalPathSimulator();

  void LoadOccupancyGrid(const nav_msgs::OccupancyGrid& msg, double depth);
  void GetSignalPath(double x1, double y1, double z1,
                     double x2, double y2, double z2,
                     std::vector<double>& segments,
                     bool compute_all_segments=true);
  private:
    bool initialized;

    double origin_x, origin_y;
    unsigned int width;
    double resolution;

    enum loss_type {FREESPACE, MATERIAL};
    typedef std::list<boost::tuple<loss_type, signed char, double> > path_loss;

    std::vector<double> segments;

    typedef CGAL::Simple_cartesian<double> K;
    typedef K::Point_3 Point;
    typedef K::Segment_3 Segment;
    typedef CGAL::Triangle_3<K> Triangle;
    typedef std::list<Triangle>::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> Traits;
    typedef CGAL::AABB_tree<Traits> Tree;
    typedef Tree::Object_and_primitive_id Object_and_primitive_id;

    unsigned int TriangleToVoxel(const Triangle t);
    std::map<signed char, double> wall_materials;

    std::map<unsigned int, unsigned int> voxel_to_triangle;
    std::vector<unsigned int> voxel_index;
    std::vector<signed char> voxel_weight;

    std::list<Triangle> triangles;
    Tree tree;
};
#endif
