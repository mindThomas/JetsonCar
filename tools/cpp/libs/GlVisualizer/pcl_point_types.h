#ifndef GL_PCL_POINT_TYPES
#define GL_PCL_POINT_TYPES

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace PointTypes
{
    struct LidarPoint
    {
        LidarPoint(float x_, float y_, float z_, uint8_t intensity_, uint8_t ring_)
            : x(x_),
              y(y_),
              z(z_),
              intensity(intensity_),
              ring(ring_)
        {
        }

        LidarPoint() :
            x(0.0),
            y(0.0),
            z(0.0),
            intensity(0),
            ring(-1)
        {
        }

        PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
        uint8_t intensity;
        uint8_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTypes::LidarPoint,           // here we assume a XYZ + "test" (as fields)
                                   (float,  x,  x)
                                   (float,  y,  y)
                                   (float,  z,  z)
                                   (uint8_t, intensity, intensity)
                                   (uint8_t, ring, ring)
)

#endif // GL_PCL_POINT_TYPES

