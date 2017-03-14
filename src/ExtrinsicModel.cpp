#include <sstream>
#include <cmath>
#include <iostream>

#include "ExtrinsicModel.hpp"
#include "xml.hpp"

bool ExtrinsicModel::GroundToImage(double xg, double yg, double zg, double &xi, double &yi, double &zi) const
{
    xg -= m_sommet[0];
    yg -= m_sommet[1];
    zg -= m_sommet[2];
    xi = m_rotation[0] * xg + m_rotation[1] * yg + m_rotation[2] * zg;
    yi = m_rotation[3] * xg + m_rotation[4] * yg + m_rotation[5] * zg;
    zi = m_rotation[6] * xg + m_rotation[7] * yg + m_rotation[8] * zg;
    return true;
}

bool ExtrinsicModel::ImageToGround(double xi, double yi, double zi, double &xg, double &yg, double &zg) const
{
    xg = m_rotation[0] * xi + m_rotation[3] * yi + m_rotation[6] * zi + m_sommet[0];
    yg = m_rotation[1] * xi + m_rotation[4] * yi + m_rotation[7] * zi + m_sommet[1];
    zg = m_rotation[2] * xi + m_rotation[5] * yi + m_rotation[8] * zi + m_sommet[2];
    return true;
}

bool ExtrinsicModel::Read(TiXmlNode* node)
{
    // Sommet
    TiXmlNode* sommet = FindNode(node,"sommet");
    m_sommet[0] = ReadNodeAs<double>(sommet,"easting");;
    m_sommet[1] = ReadNodeAs<double>(sommet,"northing");
    m_sommet[2] = ReadNodeAs<double>(sommet,"altitude");

    // Matrice
    TiXmlNode* rotation = FindNode(node,"rotation");

    m_systeme   = ReadNodeAs<std::string>(node,"systeme");
    m_grid_alti = ReadNodeAs<std::string>(node,"grid_alti");

    TiXmlNode* mat = FindNode(rotation,"mat3d");
    TiXmlNode* l1 = FindNode(mat,"l1");
    TiXmlNode* l2 = FindNode(mat,"l2");
    TiXmlNode* l3 = FindNode(mat,"l3");

    std::string Image2Ground = ReadNodeAs<std::string>(rotation,"Image2Ground");
    if ( Image2Ground == "true" ) // La matrice est de l'image vers le terrain
    {
        TiXmlNode* pt3d = FindNode(l1,"pt3d");
        m_rotation[0] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[3] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[6] = ReadNodeAs<double>(pt3d,"z");

        pt3d = FindNode(l2,"pt3d");
        m_rotation[1] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[4] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[7] = ReadNodeAs<double>(pt3d,"z");

        pt3d = FindNode(l3,"pt3d");
        m_rotation[2] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[5] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[8] = ReadNodeAs<double>(pt3d,"z");
    }
    else
    {
        TiXmlNode* pt3d = FindNode(l1,"pt3d");
        m_rotation[0] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[1] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[2] = ReadNodeAs<double>(pt3d,"z");

        pt3d = FindNode(l2,"pt3d");
        m_rotation[3] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[4] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[5] = ReadNodeAs<double>(pt3d,"z");

        pt3d = FindNode(l3,"pt3d");
        m_rotation[6] = ReadNodeAs<double>(pt3d,"x");
        m_rotation[7] = ReadNodeAs<double>(pt3d,"y");
        m_rotation[8] = ReadNodeAs<double>(pt3d,"z");

    }
    return true;
}

bool ExtrinsicModel::Write(std::ostream& out) const
{
    int prec = out.precision(2);						// Sauvegarde des parametres du flux
    std::ios::fmtflags flags = out.setf(std::ios::fixed);

    out << "  <extrinseque>" << std::endl;

    out << "   <systeme> " << m_systeme << " </systeme>"<<  std::endl;
    out << "   <grid_alti> " <<  m_grid_alti << " </grid_alti>"<<  std::endl;

    out << "   <sommet>" << std::endl;
    out << "    <easting>"  <<  m_sommet[0] << "</easting>"<<  std::endl;
    out << "    <northing>" <<  m_sommet[1] << "</northing>"<<  std::endl;
    out << "    <altitude>" <<  m_sommet[2] << "</altitude>"<<  std::endl;
    out << "   </sommet>" << std::endl;

    out.precision(6);

    out << "   <rotation>" << std::endl;
    out << "    <Image2Ground> true </Image2Ground>"<<  std::endl;
    out.precision(12);
    out << "    <mat3d>" << std::endl;
    out << "     <l1>" << std::endl;
    out << "      <pt3d>" << std::endl;
    out << "       <x>" << m_rotation[0]  << "</x>" << std::endl;
    out << "       <y>" << m_rotation[3]  << "</y>" << std::endl;
    out << "       <z>" << m_rotation[6]  << "</z>" << std::endl;
    out << "      </pt3d>" << std::endl;
    out << "     </l1>" << std::endl;
    out << "     <l2>" << std::endl;
    out << "      <pt3d>" << std::endl;
    out << "       <x>" << m_rotation[1]  << "</x>" << std::endl;
    out << "       <y>" << m_rotation[4]  << "</y>" << std::endl;
    out << "       <z>" << m_rotation[7]  << "</z>" << std::endl;
    out << "      </pt3d>" << std::endl;
    out << "     </l2>" << std::endl;
    out << "     <l3>" << std::endl;
    out << "      <pt3d>" << std::endl;
    out << "       <x>" << m_rotation[2]  << "</x>" << std::endl;
    out << "       <y>" << m_rotation[5]  << "</y>" << std::endl;
    out << "       <z>" << m_rotation[8]  << "</z>" << std::endl;
    out << "      </pt3d>" << std::endl;
    out << "     </l3> " << std::endl;
    out << "    </mat3d>" << std::endl;
    out << "   </rotation>" << std::endl;

    out << "  </extrinseque>" << std::endl;


    out.precision(prec);		// Restauration des parametres du flux
    out.setf(flags);

    return out.good();
}
