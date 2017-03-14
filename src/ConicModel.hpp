#ifndef __CONIC_MODEL_HPP__
#define __CONIC_MODEL_HPP__

#include "IntrinsicModel.hpp"
#include "DistortionPolynom.hpp"
class TiXmlNode;

class ConicModel : public IntrinsicModel
{
public:
    ConicModel() : m_distortion(NULL) {}
    ConicModel(const ConicModel &mod);

    ConicModel * Clone() const { return new ConicModel(*this); }
    ~ConicModel();

    bool GroundToImage(double x, double y, double z, double &c, double &l) const;
    bool GroundToImageAndDepth(double x, double y, double z, double &c, double &l, double &d) const;
    bool ImageToVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const;

    bool Read(TiXmlNode* node);
    bool Write(std::ostream& out) const;

    double focal() const { return m_focal; }
    void focal(double f) { m_focal = f; }
    const DistortionPolynom* distortion() const { return m_distortion; }
    DistortionPolynom* distortion() { return m_distortion; }

private:
    double m_focal;
    DistortionPolynom* m_distortion;
};

#endif // __CONIC_MODEL_HPP__
