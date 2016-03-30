#ifndef __DISTORTIONPOLYNOM_HPP__
#define __DISTORTIONPOLYNOM_HPP__

#include <ostream>
class TiXmlNode;

class DistortionPolynom
{
public:
    bool ApplyGroundToImage(double &column, double &line ) const;
    bool ApplyImageToGround(double &column, double &line ) const;

    bool Write(std::ostream& out) const;
    bool Read(TiXmlNode* node);

    double r3() const { return m_cr3; }
    double r5() const { return m_cr5; }
    double r7() const { return m_cr7; }
    double xPPS() const { return m_xPPS; }
    double yPPS() const { return m_yPPS; }

    void r3(double c) { m_cr3=c; update();}
    void r5(double c) { m_cr5=c; update();}
    void r7(double c) { m_cr7=c; update();}
    void xPPS(double x) { m_xPPS=x; }
    void yPPS(double y) { m_yPPS=y; }

private:
    void update();
    double m_xPPS, m_yPPS;
    double m_cr3, m_cr5, m_cr7;
    double m_r2_max;
};

#endif //#ifndef __DISTORTIONPOLYNOM_HPP__
