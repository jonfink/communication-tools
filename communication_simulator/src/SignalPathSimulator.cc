#include "SignalPathSimulator.h"
#include <cmath>

using namespace std;

SignalPathSimulator::SignalPathSimulator()
{
  initialized = false;

  return;
}

SignalPathSimulator::~SignalPathSimulator()
{
  return;
}

/** LoadOccupancyGrid
 * From Nathan Michael's MeshSimulator library
 */
void SignalPathSimulator::LoadOccupancyGrid(const nav_msgs::OccupancyGrid& map, double depth)
{
  tree.clear();
  triangles.clear();

  double x = map.info.origin.position.x;
  double y = map.info.origin.position.y;
  double z = map.info.origin.position.z;

  origin_x = x;
  origin_y = y;

  double delta = 0.5*map.info.resolution;
  resolution = map.info.resolution;
  width = map.info.width;

  // Iterate through the map, if a value is occupied (v > 0)
  // create a voxel of fixed depth. Voxel is defined by a set
  // of triangles, 2 per face.
  // Store an equivalent length
  // vector of voxel indices tied to triangles for randam access
  // reverse lookup of voxel given a triangle.
  unsigned int index = 0;
  vector<signed char>::const_iterator k = map.data.begin();
  for (unsigned int i = 0; i < map.info.height; i++)
    for (unsigned int j = 0; j < map.info.width; j++, ++k)
      {
        if (*k > 0)
          {
            voxel_to_triangle.insert(make_pair(i*width + j, index));

            double center_x = x + j*resolution + delta;
            double center_y = y + i*resolution + delta;

            Point face_1_1(center_x - delta, center_y + delta, z);
            Point face_1_2(center_x - delta, center_y - delta, z);
            Point face_1_3(center_x - delta, center_y - delta, z + depth);
            Point face_1_4(center_x - delta, center_y + delta, z + depth);

            // Case 1
            triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
            voxel_index.push_back(index);
            // Case 2
            triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));
            voxel_index.push_back(index);

            Point face_2_1(center_x - delta, center_y - delta, z);
            Point face_2_2(center_x + delta, center_y - delta, z);
            Point face_2_3(center_x + delta, center_y - delta, z + depth);
            Point face_2_4(center_x - delta, center_y - delta, z + depth);

            // Case 3
            triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
            voxel_index.push_back(index);
            // Case 4
            triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));
            voxel_index.push_back(index);

            Point face_3_1(center_x + delta, center_y - delta, z);
            Point face_3_2(center_x + delta, center_y + delta, z);
            Point face_3_3(center_x + delta, center_y + delta, z + depth);
            Point face_3_4(center_x + delta, center_y - delta, z + depth);

            // Case 5
            triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
            voxel_index.push_back(index);
            // Case 6
            triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));
            voxel_index.push_back(index);

            Point face_4_1(center_x + delta, center_y + delta, z);
            Point face_4_2(center_x - delta, center_y + delta, z);
            Point face_4_3(center_x - delta, center_y + delta, z + depth);
            Point face_4_4(center_x + delta, center_y + delta, z + depth);

            // Case 7
            triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
            voxel_index.push_back(index);
            // Case 8
            triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));
            voxel_index.push_back(index);

            voxel_weight.push_back(*k);
            ++index;
          }
      }

  if(triangles.size() > 0) {
    tree.rebuild(triangles.begin(), triangles.end());
    initialized = true;
  }

  return;
}

double norm(double x, double y, double z)
{
  return sqrt(x*x + y*y + z*z);
}

void SignalPathSimulator::GetSignalPath(double x1, double y1, double z1,
                                        double x2, double y2, double z2,
                                        std::vector<double>& segments,
                                        bool compute_all_segments)
{
  Point p1(x1, y1, z1);
  Point p2(x2, y2, z2);

  Segment segment_query(p1, p2);
  list<Object_and_primitive_id> intersections;
  map<unsigned int, list<double> > dist;
  vector<double> distances;
  Point last_point = p1;
  double d_segment = 0.0;
  unsigned int index = 0;

  bool intersections_exist = false;
  if(initialized)
    intersections_exist = tree.do_intersect(segment_query);

  if(intersections_exist && !compute_all_segments) {
      double dsep = norm(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
      distances.push_back(dsep/2);
      distances.push_back(dsep/2);
  }
  else if (intersections_exist)
    {
      intersections.clear();
      tree.all_intersections(segment_query,
                             std::back_inserter(intersections));

      for (list<Object_and_primitive_id>::iterator j = intersections.begin();
           j != intersections.end(); ++j)
        {
          Point point;
          if (!CGAL::assign(point, (*j).first))
            continue;

          index = TriangleToVoxel(*((*j).second));
          if (dist.count(index) == 0)
            dist.insert(make_pair(index, list<double>()));

          if (dist.count(0) == 0)
            dist.insert(make_pair(0, list<double>()));

          // double d = norm(p2.x() - point.x(),
          //                 p2.y() - point.y(),
          //                 p2.z() - point.z());

          double d_last = norm(point.x() - last_point.x(),
                               point.y() - last_point.y(),
                               point.z() - last_point.z());

          last_point = point;

          if( (d_last > sqrt(resolution*resolution+resolution*resolution)) &&
              (d_segment > 0.0) ) {
            dist[index].push_back(d_segment);
            distances.push_back(d_segment);
          }

          if(d_last > sqrt(resolution*resolution+resolution*resolution)) {
            d_segment = 0.0;

            if(d_last > 1.1*sqrt(resolution*resolution+resolution*resolution)) {
              dist[0].push_back(d_last);
              distances.push_back(d_last);
            }
          }

          if(d_last <= sqrt(resolution*resolution+resolution*resolution)) {
            d_segment += d_last;
          }
        }

      dist[index].push_back(d_segment);
      distances.push_back(d_segment);

      double d = norm(p2.x() - last_point.x(),
                      p2.y() - last_point.y(),
                      p2.z() - last_point.z());
      dist[index].push_back(d);
      distances.push_back(d);

    }

  path_loss lines;
  segments.clear();
  if (distances.size() > 0)
    {
      for(unsigned int i=0; i < distances.size(); ++i) {
        segments.push_back(distances[i]);

        if(i % 2 == 0) {
          lines.push_back(boost::make_tuple(FREESPACE, 0, distances[i]));
          ROS_DEBUG("Freespace with depth: %f\n", distances[i]);
        }
        else {
          lines.push_back(boost::make_tuple(MATERIAL, 1, distances[i]));
          ROS_DEBUG("Material with depth: %f\n", distances[i]);
        }
      }
    }
  else
    {
      // Only freespace between robots
      double dsep = norm(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
      lines.push_back(boost::make_tuple(FREESPACE, 0, dsep));
      segments.push_back(dsep);
      ROS_DEBUG("Freespace with depth: %f\n", dsep);
    }
  return;
}

unsigned int SignalPathSimulator::TriangleToVoxel(const Triangle t)
{
  double p1x = t.vertex(0).x();
  double p1y = t.vertex(0).y();
  double p2x = t.vertex(1).x();
  double p2y = t.vertex(1).y();
  double p2z = t.vertex(1).z();
  double p3z = t.vertex(2).z();

  bool same_x_12 = fabs(p1x - p2x) < 1e-10 ? true : false;
  bool same_z_23 = fabs(p2z - p3z) < 1e-10 ? true : false;

  double center_x = 0;
  double center_y = 0;
  double delta = 0.5*resolution;

  if (same_z_23) // Case 1, 3, 5, 7
    {
      if (same_x_12) // Case 1, 5
        {
          if (p1y > p2y) // Case 1
            {
              center_x = p1x + delta;
              center_y = p1y - delta;
            }
          else // Case 5
            {
              center_x = p1x - delta;
              center_y = p1y + delta;
            }
        }
      else // Case 3, 7
        {
          if (p2x > p1x) // Case 3
            {
              center_x = p1x + delta;
              center_y = p1y + delta;
            }
          else // Case 7
            {
              center_x = p1x - delta;
              center_y = p1y - delta;
            }
        }
    }
  else // Case 2, 4, 6, 8
    {
      if (same_x_12) // Case 2, 6
        {
          if (p1y > p2y) // Case 2
            {
              center_x = p1x + delta;
              center_y = p1y - delta;
            }
          else // Case 6
            {
              center_x = p1x - delta;
              center_y = p1y + delta;
            }
        }
      else // Case 4, 8
        {
          if (p1x < p2x) // Case 4
            {
              center_x = p1x + delta;
              center_y = p1y + delta;
            }
          else // Case 8
            {
              center_x = p1x - delta;
              center_y = p1y - delta;
            }
        }
    }

  double j = (center_x - origin_x - delta)/resolution;
  double i = (center_y - origin_y - delta)/resolution;

  if (voxel_to_triangle.count(i*width + j) == 0)
    {
      puts("failed to find index");
      return 0;
    }

  return voxel_to_triangle[i*width + j];
}
