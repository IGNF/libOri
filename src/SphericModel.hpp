#ifndef __SPHERIC_MODEL_HPP__
#define __SPHERIC_MODEL_HPP__

#include "IntrinsicModel.hpp"

class SphericModel : public IntrinsicModel
{
public:

  #if HAVE_XML
      bool Read(TiXmlNode* node);
      bool Write(std::ostream& out) const;
  #endif

  #if HAVE_JSON
      bool Read(const Json::Value& json, double position[3], double rotation[9], int& orientation);
  #endif

    bool GroundToImage(double x, double y, double z, double &c, double &l) const;
    bool GroundToImageAndDepth(double x, double y, double z, double &c, double &l, double &d) const;
    bool ImageToVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1,double &z1) const;

    virtual SphericModel * Clone() const { return new SphericModel(*this); }

    double lambdaMin() const { return m_lambdaMin; }
    double lambdaMax() const { return m_lambdaMax; }
    double phiMin() const { return m_phiMin; }
    double phiMax() const { return m_phiMax; }

    void lambdaMin(double x) { m_lambdaMin=x; }
    void lambdaMax(double x) { m_lambdaMax=x; }
    void phiMin(double x) { m_phiMin=x; }
    void phiMax(double x) { m_phiMax=x; }
private:
    double m_lambdaMin, m_lambdaMax;
    double m_phiMin, m_phiMax;
};

#endif // __SPHERIC_MODEL_HPP__
