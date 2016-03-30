#include <iostream>
#include <sstream>
#include <cmath>
#include <fstream>

#include "ConicModel.hpp"
#include "SphericModel.hpp"
#include "Ori.hpp"
#include "xml.hpp"

//-----------------------------------------------------------------------------
Ori::Ori() : m_intrinsic(NULL) {}
//-----------------------------------------------------------------------------
Ori::Ori(const Ori& ori)
    : m_intrinsic(NULL), m_extrinsic(ori.m_extrinsic)
{
    if ( ori.m_intrinsic ) m_intrinsic = ori.m_intrinsic->Clone();
}
//-----------------------------------------------------------------------------
Ori::~Ori()
{
    if ( m_intrinsic ) delete m_intrinsic;
}
//-----------------------------------------------------------------------------
bool Ori::Read( const std::string &file )
{
    TiXmlDocument* doc = XmlOpen(file);
    if(!doc) return false;
    TiXmlNode* root = XmlRoot(doc,"orientation");
    bool ok = (root!=NULL);

    std::string versionNode = ReadNodeAsString(root,"version");
    if ( ok && versionNode != "1.0" )
    {
        std::cerr  << "ERROR: Ori 'version' MUST be '1.0' ! Value is '" << versionNode << "'." << std::endl;
        ok = false;
    }
    
    TiXmlNode* geometry = FindNode(root,"geometry");
    TiXmlNode* extrinsic = FindNode(geometry,"extrinseque");
    ok &= m_extrinsic.Read(extrinsic);
    
    TiXmlNode* intrinsic =  FindNode(geometry,"intrinseque");
    if(FindNode(intrinsic,"spherique"))
        m_intrinsic = new SphericModel();
    else
        m_intrinsic = new ConicModel();
    
    ok &= m_intrinsic->Read(intrinsic);
    
    XmlClose(doc);
    return ok;
}
//-----------------------------------------------------------------------------
bool Ori::Write(const std::string& filename) const
{
    std::ofstream out(filename.c_str());
    return Write(out);
}

bool Ori::Write(std::ostream& out) const
{
    out << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << std::endl;
    out << "<?xml-stylesheet type=\"text/xsl\" href=\"./XSLT/orientation_1_0.xsl\"?>" << std::endl;
    out << "<orientation>" << std::endl;
    out << " <version> 1.0 </version>" << std::endl;
    out << " <geometry>" << std::endl;
    
    bool ok = m_extrinsic.Write(out);
    if(m_intrinsic) ok &= m_intrinsic->Write(out);
    
    out << " </geometry>" << std::endl;
    out << "</orientation>" << std::endl;
    return ok && out.good();
}
//-----------------------------------------------------------------------------
bool Ori::GroundToImage( double x, double y, double z, double &c, double &l ) const
{
    double gx, gy, gz;
    return     m_extrinsic.GroundToImage( x , y , z , gx , gy , gz )
            && m_intrinsic->GroundToImage( gx, gy, gz, c  , l );
}
//-----------------------------------------------------------------------------
bool Ori::ImageAndDepthToGround(double c, double l, double d, double &x, double &y, double &z) const
{
    double x0, y0, z0, x1, y1, z1;
    if(!ImageToGroundVec(c,l, x0,y0,z0, x1, y1, z1)) return false;
    double dx = x1-x0;
    double dy = y1-y0;
    double dz = z1-z0;
    double norm = sqrtf(dx*dx+dy*dy+dz*dz);
    if(norm==0) return false;
    double lambda = d/norm;
    x = x0 + (x1-x0) * lambda;
    y = y0 + (y1-y0) * lambda;
    z = z0 + (z1-z0) * lambda;
    return true;
}

//-----------------------------------------------------------------------------
bool Ori::ImageToGroundVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const
{
    return     m_intrinsic->ImageToVec(c,l, x0,y0,z0, x1,y1,z1)
            && m_extrinsic.ImageToGround(x0,y0,z0, x0,y0,z0)
            && m_extrinsic.ImageToGround(x1,y1,z1, x1,y1,z1);
}
