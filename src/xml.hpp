#if HAVE_XML
#ifndef XML_HPP
#define XML_HPP

#include <sstream>
class TiXmlNode;
class TiXmlDocument;

class XmlDoc
{
public:
  XmlDoc(const std::string& filename);
  ~XmlDoc();

  TiXmlNode* root(const std::string& tag);

private:
  TiXmlDocument *m_doc;
};

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
#endif // HAVE_XML
