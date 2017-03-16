#include <iostream>
#include <sstream>
#include <cmath>
#include <fstream>

#include "Ori.hpp"
#include "IntrinsicModel.hpp"

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

#if HAVE_XML
#include "xml.hpp"
#include "ConicModel.hpp"
#include "SphericModel.hpp"

bool Ori::Read( const std::string &file )
{
    XmlDoc doc(file);
    TiXmlNode* root = doc.root("orientation");
    if(!root) return false;

    std::string versionNode = ReadNodeAsString(root,"version");
    if ( versionNode != "1.0" )
    {
      std::cerr  << "ERROR: Ori 'version' MUST be '1.0' ! Value is '" << versionNode << "'." << std::endl;
      return false;
    }

    TiXmlNode* geometry = FindNode(root,"geometry");
    TiXmlNode* extrinsic = FindNode(geometry,"extrinseque");
    if(! m_extrinsic.Read(extrinsic))
      return false;

    TiXmlNode* intrinsic =  FindNode(geometry,"intrinseque");
    if(! (m_intrinsic = IntrinsicModel::New(intrinsic) ) )
      return false;

    return true;
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

#endif // HAVE_XML

#if HAVE_JSON
#include <json/json.h>

bool Ori::Read (
      const std::string &camera_filename, int camera_id,
      const std::string &panoramic_filename, int panoramic_id
)
{
    Json::Value camera;   // will contains the camera root value after parsing.
    Json::Value panoramic;   // will contains the panoramic root value after parsing.
    Json::Reader reader;
    std::ifstream camera_file(camera_filename.c_str(), std::ifstream::binary);
    if(!(camera_file.good() && reader.parse( camera_file, camera, false )))
    {
      std::cerr << "Error reading " << camera_filename <<"\n";
      std::cerr << reader.getFormatedErrorMessages() << "\n";
      return false;
    }
    std::ifstream panoramic_file(panoramic_filename.c_str(), std::ifstream::binary);
    if(!(panoramic_file.good() && reader.parse( panoramic_file, panoramic, false )))
    {
      std::cerr << "Error reading " << panoramic_filename <<"\n";
      std::cerr << reader.getFormatedErrorMessages() << "\n";
      return false;
    }
    double position[3];
    double rotation[9];
    int orientation;
    m_intrinsic = IntrinsicModel::New(camera[camera_id],position,rotation,orientation);
    return m_intrinsic!=NULL && m_extrinsic.Read(panoramic[panoramic_id],position,rotation,orientation);
}
#endif // HAVE_JSON

//-----------------------------------------------------------------------------
bool Ori::GroundToImage( double x, double y, double z, double &c, double &l ) const
{
    double gx, gy, gz;
    return     m_extrinsic.GroundToImage( x , y , z , gx , gy , gz )
            && m_intrinsic->GroundToImage( gx, gy, gz, c  , l );
}
//-----------------------------------------------------------------------------
bool Ori::GroundToImageAndDepth( double x, double y, double z, double &c, double &l, double &d ) const
{
    double gx, gy, gz;
    return     m_extrinsic.GroundToImage( x , y , z , gx , gy , gz )
            && m_intrinsic->GroundToImageAndDepth( gx, gy, gz, c  , l , d  );
}
//-----------------------------------------------------------------------------
bool Ori::ImageAndDepthToGround(double c, double l, double d, double &x, double &y, double &z) const
{
    double x0, y0, z0, x1, y1, z1;
    if(!ImageToGroundVec(c,l, x0,y0,z0, x1, y1, z1)) return false;
    x = x0 + (x1-x0) * d;
    y = y0 + (y1-y0) * d;
    z = z0 + (z1-z0) * d;
    return true;
}

//-----------------------------------------------------------------------------
bool Ori::ImageToGroundVec(double c, double l, double &x0, double &y0, double &z0, double &x1, double &y1, double &z1) const
{
    return     m_intrinsic->ImageToVec(c,l, x0,y0,z0, x1,y1,z1)
            && m_extrinsic.ImageToGround(x0,y0,z0, x0,y0,z0)
            && m_extrinsic.ImageToGround(x1,y1,z1, x1,y1,z1);
}
