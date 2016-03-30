#ifndef XML_HPP
#define XML_HPP

#include <sstream>
class TiXmlNode;
class TiXmlDocument;

TiXmlDocument* XmlOpen(const std::string& filename);
void XmlClose(TiXmlDocument* doc);

TiXmlNode* XmlRoot(TiXmlDocument *doc, const std::string& tag);

TiXmlNode* FindNode(TiXmlNode* node, const std::string& nodename);

std::string ReadNodeAsString(TiXmlNode* node, const std::string& nodename);

template <typename T>
        T ReadNodeAs(TiXmlNode* node, const std::string& nodename)
{
    std::string str = ReadNodeAsString(node,nodename);
    std::istringstream iss(str);
    T res;
    iss >> res;
    return res;
}

#endif // XML_HPP
