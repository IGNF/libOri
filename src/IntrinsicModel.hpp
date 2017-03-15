#ifndef __INTRINSIC_MODEL_HPP_
#define __INTRINSIC_MODEL_HPP_

#include <iostream>
class TiXmlNode;
namespace Json { class Value; }

class IntrinsicModel
{
public:
    virtual ~IntrinsicModel() {}
    virtual IntrinsicModel * Clone() const = 0;

#if HAVE_XML
    static IntrinsicModel *New(TiXmlNode* node);
    virtual bool Read(TiXmlNode* node) = 0;
    virtual bool Write(std::ostream& out) const = 0;
#endif

#if HAVE_JSON
    static IntrinsicModel *New(const Json::Value& json, double position[3], double rotation[9], int& orientation);
    virtual bool Read(const Json::Value& node, double position[3], double rotation[9], int& orientation) = 0;
#endif

    virtual bool GroundToImage(double x, double y, double z, double &c, double &l) const = 0;
    virtual bool GroundToImageAndDepth(double x, double y, double z, double &c, double &l, double &d) const = 0;
    virtual bool ImageToVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const = 0;

    unsigned int width() const { return m_width; }
    unsigned int height() const { return m_height; }
    double cPPA() const { return m_cPPA; }
    double lPPA() const { return m_lPPA; }

    void width(unsigned int w) { m_width=w; }
    void height(unsigned int h) { m_height=h; }
    void cPPA(double c) { m_cPPA=c; }
    void lPPA(double l) { m_lPPA=l; }

protected:
    unsigned int m_width, m_height;
    double m_cPPA, m_lPPA;
};

#endif /*__INTRINSIC_MODEL_HPP_*/
