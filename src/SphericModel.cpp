#include <cmath>
#include <iostream>
#include <sstream>
#include "SphericModel.hpp"
#include "xml.hpp"

//-----------------------------------------------------------------------------
bool SphericModel::GroundToImage(double x, double y, double z, double &c, double &l) const
{
        double norm   = sqrt(x*x + y*y + z*z);
        double lambda = m_lambdaMax - atan2(y,x);
        double phi    = m_phiMax    - acos(z/norm);
	c = m_width * lambda / (m_lambdaMax - m_lambdaMin );
	l = m_lPPA - m_height * phi / (m_phiMax - m_phiMin );
        return true;
}
//-----------------------------------------------------------------------------
bool SphericModel::GroundToImageAndDepth(double x, double y, double z, double &c, double &l, double &d) const
{
        d   = sqrt(x*x + y*y + z*z);
        double lambda = m_lambdaMax - atan2(y,x);
        double phi    = m_phiMax    - acos(z/d);
	c = m_width * lambda / (m_lambdaMax - m_lambdaMin );
	l = m_lPPA - m_height * phi / (m_phiMax - m_phiMin );
        return true;
}
//-----------------------------------------------------------------------------
bool SphericModel::ImageToVec(double c, double l, double &x0, double &y0, double &z0,
		double &x1, double &y1, double &z1) const
{
	x0 = y0 = z0 = 0.;

	double lambda = m_lambdaMin + (m_lambdaMax - m_lambdaMin) * c / m_width;
        double phi    = m_phiMax    + (m_phiMin    - m_phiMax   ) * l / m_height;
	x1 = cos(phi) * cos(-lambda);
	y1 = cos(phi) * sin(-lambda);
	z1 = sin(phi);
        return true;
}
//-----------------------------------------------------------------------------
bool SphericModel::Read(TiXmlNode* node)
{
        TiXmlNode* spherique = FindNode(node,"spherique");
        TiXmlNode* image_size = FindNode(spherique,"image_size");
        m_width  = ReadNodeAs<unsigned int>(image_size,"width");
        m_height = ReadNodeAs<unsigned int>(image_size,"height");

        TiXmlNode* frame = FindNode(spherique,"frame");
        m_lambdaMin= ReadNodeAs<double>(frame, "lambda_min");
        m_lambdaMax= ReadNodeAs<double>(frame, "lambda_max");
        m_phiMin   = ReadNodeAs<double>(frame, "phi_min");
        m_phiMax   = ReadNodeAs<double>(frame, "phi_max");

        TiXmlNode* ppa = FindNode(spherique,"ppa");
        m_cPPA = ReadNodeAs<double>(ppa, "c");
        m_lPPA = ReadNodeAs<double>(ppa, "l");

	return true;
}
//-----------------------------------------------------------------------------
bool SphericModel::Write(std::ostream& out) const
{
    int prec = out.precision(3);						// Sauvegarde des parametres du flux
    std::ios::fmtflags flags = out.setf(std::ios::fixed);

    out << "  <intrinseque>" << std::endl;

    out << "   <spherique>" << std::endl;
    out << "    <image_size> " << std::endl;
    out << "     <width> " <<  m_width << " </width>" << std::endl;
    out << "     <height> " <<  m_height << " </height>" << std::endl;
    out << "    </image_size>" << std::endl;

    out.precision(12);

    out << "    <frame>" << std::endl;
    out << "     <lambda_min> " <<  m_lambdaMin << " </lambda_min>" << std::endl;
    out << "     <lambda_max> " <<  m_lambdaMax  << " </lambda_max>" << std::endl;
    out << "     <phi_min> " <<  m_phiMin << " </phi_min>" << std::endl;
    out << "     <phi_max> " <<  m_phiMax  << " </phi_max>" << std::endl;
    out << "    </frame>" << std::endl;

    out.precision(3);

    out << "    <ppa>" << std::endl;
    out << "     <c> " <<  m_cPPA << " </c>" << std::endl;
    out << "     <l> " <<  m_lPPA  << " </l>" << std::endl;
    out << "    </ppa>" << std::endl;

    out << "   </spherique>" << std::endl;

    out << "  </intrinseque>" << std::endl;

    out.precision(prec);		// Restauration des parametres du flux
    out.unsetf(std::ios::fixed);
    out.setf(flags);

    return out.good();
}
