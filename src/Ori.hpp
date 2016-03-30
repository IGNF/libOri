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

    bool Read ( const std::string &file );
    bool Write( const std::string &file ) const;
    bool Write(std::ostream& out) const;

    bool GroundToImage( double x, double y, double z, double &c, double &l ) const;
    bool ImageToGroundVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const;
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
