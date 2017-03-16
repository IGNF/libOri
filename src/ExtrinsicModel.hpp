#ifndef __EXTRINSIC_MODEL_HPP__
#define __EXTRINSIC_MODEL_HPP__

class TiXmlNode;
namespace Json { class Value; }

#include <ostream>

class ExtrinsicModel
{
public:
    bool GroundToImage(double xg, double yg, double zg, double &xi, double &yi, double &zi) const;
    bool ImageToGround(double xi, double yi, double zi, double &xg, double &yg, double &zg) const;

#if HAVE_XML
    bool Read(TiXmlNode* node);
    bool Write(std::ostream& out) const;
#endif

#if HAVE_JSON
    bool Read(const Json::Value& json, const double position[3], const double rotation[9], int orientation);
#endif

    double *sommet() { return m_sommet; }
    const double *sommet() const { return m_sommet; }

    double *rotation() { return m_rotation; }
    const double *rotation() const { return m_rotation; }

    const std::string& systeme() const { return m_systeme; }
    const std::string& grid_alti() const { return m_grid_alti; }
    void systeme(const std::string& s) { m_systeme = s; }
    void grid_alti(const std::string& s) { m_grid_alti = s; }

private:
    double m_sommet[3];
    double m_rotation[9];
    std::string m_systeme, m_grid_alti;
};

#endif // __EXTRINSIC_MODEL_HPP__
