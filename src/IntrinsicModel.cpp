
#include "IntrinsicModel.hpp"
#include "ConicModel.hpp"
#include "SphericModel.hpp"

#if HAVE_XML
#include "xml.hpp"

IntrinsicModel *IntrinsicModel::New(TiXmlNode* node)
{
  IntrinsicModel *intrinsic = NULL;
  if(FindNode(node,"spherique"))
    intrinsic = new SphericModel();
  else if(FindNode(node,"sensor"))
    intrinsic = new ConicModel();

  if(intrinsic && !intrinsic->Read(node))
  {
    delete intrinsic;
    return NULL;
  }

  return intrinsic;
}

#endif // HAVE_XML

#if HAVE_JSON
#include <json/json.h>

IntrinsicModel *IntrinsicModel::New(const Json::Value& json, double position[3], double rotation[9], int& orientation)
{
  IntrinsicModel *intrinsic = new ConicModel();
  if(intrinsic && !intrinsic->Read(json, position, rotation, orientation))
  {
    delete intrinsic;
    return NULL;
  }
  return intrinsic;
}
#endif // HAVE_JSON
