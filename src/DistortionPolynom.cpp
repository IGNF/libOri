#include <sstream>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include "xml.hpp"

#include "DistortionPolynom.hpp"
#define DPIGN_ERR2MAX 1e-15

// a set of functions to find the validity domain of the distortion polynom
// which is from 0 (obviously) to sqrt(R) where R is the first positive root of (1+c1)+3c3R+5c5R^2+7c7R^3
int sgn(double x) { return (x>0) - (x<0); }
double cubic_root(double x)
{
    return sgn(x)*pow(fabs(x),1./3.);
}
std::vector<double> quadratic_roots(double a, double b, double c)
{
    std::vector<double> ret;
    double delta = b*b-4*a*c;
    if(delta<0) return ret;
    double x0 = -b/(2*a);
    if(delta==0) {ret.push_back(x0); return ret;}
    double sqr_delta_2a = sqrt(delta)/(2*a);
    ret.push_back(x0-sqr_delta_2a);
    ret.push_back(x0+sqr_delta_2a);
    return ret;
}
std::vector<double> cardan_cubic_roots(double a, double b, double c, double d)
{
    if(a==0.) return quadratic_roots(b,c,d);
    std::vector<double> ret;
    double vt=-b/(3*a);
    double a2 = a*a;
    double b2 = b*b;
    double a3 = a*a2;
    double b3 = b*b2;
    double p=c/a-b2/(3*a2);
    double q=b3/(a3*13.5)+d/a-b*c/(3*a2);
    if(p==0.)
    {
        double x0 =cubic_root(-q)+vt;
        ret.push_back(x0);
        ret.push_back(x0);
        ret.push_back(x0);
        return ret;
    }
    double p3_4_27=p*p*p*4/27;
    double del=q*q+p3_4_27;
    if(del > 0.)
    {
        double sqrt_del=sqrt(del);
        double u=cubic_root((-q+sqrt_del)/2);
        double v=cubic_root((-q-sqrt_del)/2);
        ret.push_back(u+v+vt);
    }
    else if (del==0.)
    {
        double z0 =3*q/p;
        double x0 = vt + z0;
        double x12= vt - z0*0.5;
        ret.push_back(x0);
        ret.push_back(x12);
        ret.push_back(x12);
    }
    else // (del < 0.)
    {
        double kos=acos(-q/sqrt(p3_4_27));
        double r=2*sqrt(-p/3);
        ret.push_back(r*cos((kos     )/3)+vt);
        ret.push_back(r*cos((kos+  M_PI)/3)+vt);
        ret.push_back(r*cos((kos+2*M_PI)/3)+vt);
    }
    return ret;
}
double first_cubic_root(double a, double b, double c, double d)
{
    std::vector<double> roots = cardan_cubic_roots(a, b, c, d);
    std::sort(roots.begin(),roots.end());
    for(std::vector<double>::iterator it=roots.begin(); it != roots.end(); it++)
        if(*it > 0.) return *it;
    return std::numeric_limits<double>::infinity();
}

bool DistortionPolynom::ApplyImageToGround(double &column, double &line) const
{
    // we want to invert p'=p+Q(||p-p0||^2).(p-p0)=p0+(1+Q(||p-p0||^2)).(p-p0)
    // with p=(column,line), p0=PPS
    // and  Q(x)=x*(m_cr3 + x*(m_cr5 + x*m_cr7))

    double x = column - m_xPPS;
    double y = line - m_yPPS;
    double r2 = x*x + y*y;
    double r4 = r2*r2;
    double r6 = r4*r2;

    // introducing t s.t. (p-p0)=t(p'-p0) and r2=||p'-p0||^2,
    // we thus want to find the root t of R(t)=-1+t+tQ(t^2.r2)
    // coefficients of R (t) =-1+t+c3.t^3+c5.t^5+c7.t^7
    // coefficients of R'(t) = 1+d2.t^2+d4.t^4+d6.t^6
    double c3 = m_cr3*r2, d2 = 3*c3;
    double c5 = m_cr5*r4, d4 = 5*c5;
    double c7 = m_cr7*r6, d6 = 7*c7;

    double t = 1; // Newton's method initialisation
    int i_max=50; // max num of iterations
    for(int i=0; i<i_max; ++i)
    {
        double t2 = t *t;
        double R  = -1+t+t*t2*(c3+t2*(c5+t2*c7));
//        std::cout << "-->" << i << ": error = " << sqrt(R*R*r2) << std::endl;
        if(R*R*r2<DPIGN_ERR2MAX && t2*r2<m_r2_max) // success !
        {
            column = m_xPPS + t*x;
            line   = m_yPPS + t*y;
            return true;
        }
        double R_ =    1+  t2*(d2+t2*(d4+t2*d6));
        t -= R/R_; // Newton's method iteration
        if(t<0) t=0;
    }
    std::cerr << "DistortionPolynom's polynom inversion failed, leaving the point undistorded..." << std::endl;
    return false;
}


bool DistortionPolynom::ApplyGroundToImage(double &column, double &line) const
{
    double x = column - m_xPPS;
    double y = line - m_yPPS;
    double r2 = x*x + y*y;
    if (r2 > m_r2_max) return false; // zone de validite du polynome
    double dr = r2*(m_cr3 + r2*(m_cr5 + r2*m_cr7));
    column += dr * x;
    line   += dr * y;
    return true;
}

//-----------------------------------------------------------------------------
bool DistortionPolynom::Write(std::ostream& out) const
{
    int prec = out.precision(3);						// Sauvegarde des parametres du flux
    std::ios::fmtflags flags = out.setf(std::ios::fixed);

    out << "    <distortion>" << std::endl;
    out << "     <pps>" << std::endl;
    out << "      <c> " <<  m_xPPS << "</c>" << std::endl;
    out << "      <l> " <<  m_yPPS << "</l>" << std::endl;
    out << "     </pps>" << std::endl;

    out.unsetf(std::ios::fixed);
    out.setf(std::ios::scientific);
    out.precision(7);

    out << "     <r3> " <<  m_cr3 << "</r3>" << std::endl;
    out << "     <r5> " <<  m_cr5 << "</r5>" << std::endl;
    out << "     <r7> " <<  m_cr7 << "</r7>" << std::endl;
    out << "    </distortion>" << std::endl;

    out.precision(prec);		// Restauration des parametres du flux
    out.setf(flags);

    return out.good();
}

void DistortionPolynom::update()
{
    m_r2_max = first_cubic_root(7*m_cr7,5*m_cr5,3*m_cr3,1);
}

//-----------------------------------------------------------------------------
bool DistortionPolynom::Read(TiXmlNode* node)
{
    TiXmlNode* pps = FindNode(node,"pps");
    m_xPPS = ReadNodeAs<double>(pps,"c");
    m_yPPS = ReadNodeAs<double>(pps,"l");
    m_cr3  = ReadNodeAs<double>(node,"r3");
    m_cr5  = ReadNodeAs<double>(node,"r5");
    m_cr7  = ReadNodeAs<double>(node,"r7");

    update();
    return true;
}
