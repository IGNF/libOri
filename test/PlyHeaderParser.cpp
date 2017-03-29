
// m3v
#include "PlyHeaderParser.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

#define ERR(txt) {cout << "[" << __FUNCTION__ << "] ERROR: " << txt << endl; return false;}
#define LOG(txt) {cout << txt;}
#define LLOG(txt) {cout << "[" << __FUNCTION__ << "] " << txt << endl;}

unsigned int TypeSize(string type)
{
    if(type == "double" || type == "float32") return sizeof(double);
    if(type  == "float" || type == "float32") return sizeof(float);
    if(type  == "int" || type == "int32") return sizeof(int);
    if(type  == "uint" || type == "uint32") return sizeof(unsigned int);
    if(type  == "short" || type == "int16") return sizeof(short);
    if(type  == "ushort" || type == "uint16") return sizeof(unsigned short);
    if(type  == "char" || type == "int8") return sizeof(char);
    if(type  == "uchar" || type == "uint8") return sizeof(unsigned char);
    LLOG("Unknown type " << type);
    return 0;
}

unsigned int PlyProperty::Size()
{
    return TypeSize(m_type);
}

unsigned int PlyProperty::ListSizeSize()
{
    return TypeSize(m_list_size_type);
}

unsigned int PlyProperty::ListElemSize()
{
    return TypeSize(m_list_elem_type);
}

// Getline robust to windows line ends
inline void RobustGetLine(ifstream & ifs, string & line)
{
    getline(ifs, line);
    if(line[line.size()-1] == '\r')
        line.erase(line.size()-1);
}

bool PlyHeader::Parse(string ply_filename)
{
    // open file
    ifstream ifs(ply_filename.c_str());
    if(!ifs.good()) ERR("Failed to open " << ply_filename)
            string line;
    RobustGetLine(ifs, line);
    { // line == "ply" does not always work (weird char at end of line)
        istringstream iss(line);
        string word;
        iss >> word;
        if(word != "ply") ERR("not a PLY file: starts with " << word);
    }
    LLOG("Loading ply header from " << ply_filename);
    int i_line=0;
    bool end_reached = false;
    string word="";
    while(!ifs.eof() && !end_reached && i_line++<1000)
    {
        RobustGetLine(ifs, line);
        LOG(line << "->");
        istringstream iss(line);
        word="";
        iss >> word;
        if(word == "format")
        {
            iss >> m_format.m_name;
            iss >> m_format.m_version;
            LOG("Format and version stored");
        }
        else if(word == "comment")
        {
            mv_comment.push_back(line);
            iss >> word;
            if(word == "IGN")
            {
                iss >> word;
                if(word == "offset" || word == "Offset")
                {
                    iss >> word;
                    if(word == "GPS")
                    {
                        LOG("GPS Offset not handled");
                    }
                    else if(word == "Pos")
                    {
                        iss >> m_georef.m_E >> m_georef.m_N >> m_georef.m_H;
                        LOG("->E=" << m_georef.m_E << ", N=" << m_georef.m_N << ", H=" << m_georef.m_H);
                    }
                    else LOG("unknown IGN Offset");
                }
                else LOG("unknown IGN comment");
            }
            else LOG("comment stored");
        }
        else if(word == "element")
        {
            PlyElement element;
            iss >> element.m_name >> element.m_num;
            mv_element.push_back(element);
            LOG(element.m_num << " " << element.m_name << " elements added");
        }
        else if(word == "property")
        {
            PlyProperty property;
            iss >> property.m_type;
            if(property.m_type == "list") // list types have 3 words: list $size_type $prop_type
            {
                iss >> property.m_list_size_type;
                iss >> property.m_list_elem_type;
            }
            iss >> property.m_name;
            if(mv_element.empty()) ERR("Property without element !!!")
            mv_element.back().mv_property.push_back(property);
            LOG(mv_element.back().m_name << " property " << property.m_name << " of type " << property.m_type <<" added");
        }
        else if(word == "end_header")
        {
            LOG("stopping");
            end_reached = true;
        }
        else
        {
            LOG("not recognized");
        }
        LOG(endl);
    }
    ifs.close();
}
