#include "tinyxml.h"
#include <sstream>
#include <iostream>
#include "xml.hpp"

TiXmlDocument* XmlOpen(const std::string& filename)
{
    TiXmlDocument *doc = new TiXmlDocument( filename.c_str() );
    if ( doc->LoadFile() ) return doc;
    std::cerr << "ERROR: Unable to open file " << filename << " !" << std::endl;
    delete doc;
    return NULL;
}

void XmlClose(TiXmlDocument* doc)
{
    if (doc) delete doc;
}

TiXmlNode* XmlRoot(TiXmlDocument *doc, const std::string& tag)
{
    if(!doc) return NULL;
    TiXmlNode* root = doc->RootElement();
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
