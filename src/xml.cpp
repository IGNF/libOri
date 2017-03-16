#if HAVE_XML

#include "tinyxml.h"
#include <sstream>
#include <iostream>
#include "xml.hpp"

XmlDoc::XmlDoc(const std::string& filename)
{
    m_doc = new TiXmlDocument( filename.c_str() );
    if ( ! m_doc->LoadFile() )
    {
      delete m_doc;
      m_doc = NULL;
      std::cerr << "ERROR: Unable to open file " << filename << " !" << std::endl;
    }
}

XmlDoc::~XmlDoc()
{
    if (m_doc) delete m_doc;
}

TiXmlNode* XmlDoc::root(const std::string& tag)
{
    if(!m_doc) return NULL;
    TiXmlNode* root = m_doc->RootElement();
    std::string filetag = root->Value();
    if(filetag==tag) return root;
    std::cerr << "ERROR: Main tag is "<< filetag<< " ! (should be "<< tag << ")" << std::endl;
    return NULL;
}

TiXmlNode* FindNode(TiXmlNode* node, const std::string& nodename)
{
    return (node) ? node->FirstChild(nodename.c_str()) : NULL;
}

std::string ReadNodeAsString(TiXmlNode* node, const std::string& nodename)
{
    TiXmlNode* n = FindNode(node,nodename);
    const char* val = (n) ? n->ToElement()->GetText() : "";
    return val;
}

#endif // HAVE_XML
