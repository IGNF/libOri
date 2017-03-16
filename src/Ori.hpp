#ifndef __ORI_HPP__
#define __ORI_HPP__

#include "ExtrinsicModel.hpp"
class IntrinsicModel;

class Ori
{
public:
    Ori();
    Ori(const Ori& ori);
    virtual ~Ori();

#if HAVE_XML
    /// Reads this orientation from a .ori.xml file
    /// returns true if read is successful
    bool Read ( const std::string &file );

    /// Writes this orientation to a .ori.xml file
    /// returns true if write is successful
    bool Write( const std::string &file ) const;

    /// Writes this orientation to a stream
    /// returns true if write is successful
    bool Write(std::ostream& out) const;
#endif

#if HAVE_JSON
    /// Reads this orientation from an itowns json file
    /// returns true if read is successful
    bool Read (
      const std::string &camera_file, int camera_id,
      const std::string &panoramic_file, int panoramic_id
    );
#endif

    /// Gets the (c,l) image coordinates of a 3D point of coordinates (x,y,z)
    /// returns false if the point does not project to the image
    bool GroundToImage( double x, double y, double z, double &c, double &l ) const;

    /// Gets the (c,l) image coordinates and the depth d of a 3D point of coordinates (x,y,z)
    /// returns false if the point does not project to the image
    bool GroundToImageAndDepth( double x, double y, double z, double &c, double &l, double& d ) const;

    /// for a point at image coordinates (c,l) gives the projection center and 3D coordinates of the corresponding point in focal plane
    bool ImageToGroundVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const;

    /// Gets the 3D point at depth d for a point at image coordinates (c,l)
    bool ImageAndDepthToGround(double c, double l, double d, double &x, double &y, double &z) const;

    const IntrinsicModel* intrinsic() const { return m_intrinsic; }
    const ExtrinsicModel& extrinsic() const { return m_extrinsic; }

    IntrinsicModel* intrinsic() { return m_intrinsic; }
    ExtrinsicModel& extrinsic() { return m_extrinsic; }

protected:
    IntrinsicModel *m_intrinsic;
    ExtrinsicModel  m_extrinsic;
};

#endif // __ORI_HPP__
