/**
* @file
* m3v (Matis 3D Viewer) ParsePlyHeader
**/

#pragma once

#include <vector>
#include <string>

void RobustGetLine(std::ifstream & ifs, std::string & line);

class PlyFormat
{
public:
    std::string m_name, m_version;
    PlyFormat(std::string name="", std::string version=""):m_name(name), m_version(version){}
};

class PlyComment
{
public:
    std::string m_comment;
    PlyComment(std::string comment=""):m_comment(comment){}
};

class PlyIgnGeoref
{
public:
    double m_E, m_N, m_H;
    PlyIgnGeoref(double E=0., double N=0., double H=0.):m_E(E), m_N(N), m_H(H){}
};

class PlyProperty
{
public:
    //enum Type{eDouble, eFloat, eInt, eUInt, eShort, eUShort, eChar, eUChar};
    std::string m_type, m_name, m_list_size_type, m_list_elem_type;
    PlyProperty(std::string type="", std::string name=""):
        m_type(type), m_name(name), m_list_size_type(""), m_list_elem_type(""){}
    unsigned int Size();
    unsigned int ListSizeSize();
    unsigned int ListElemSize();
};

class PlyElement
{
public:
    std::string m_name;
    unsigned int m_num;
    std::vector<PlyProperty> mv_property;
    PlyElement(std::string name="", unsigned int num=0):m_name(name),m_num(num){}
};

class PlyHeader
{
    public:
    std::vector<PlyElement> mv_element;
    std::vector<PlyComment> mv_comment;
    PlyIgnGeoref m_georef;
    PlyFormat m_format;
    PlyHeader(){}
    bool Parse(std::string ply_filename);
    PlyHeader(std::string ply_filename){Parse(ply_filename);}
};
