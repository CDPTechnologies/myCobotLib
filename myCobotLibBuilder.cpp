/**
myCobotLibBuilder implementation.
*/

#include "myCobotIO.h"
#include "myCobotLibBuilder.h"

using namespace myCobotLib;

/*!
  \inmodule myCobotLib
  \namespace myCobotLib

  \brief Contains myCobotLib implementation classes.
  Click into each class to see the documentation for that class, or search for a keyword in the help documentation.
*/

/**
\internal
Do not edit. Autogenerated Builder constructor.
*/
myCobotLibBuilder::myCobotLibBuilder(const char* libName)
  : CDPBuilder(libName, __DATE__ " " __TIME__)
{
}

/**
\internal
Do not edit. Autogenerated Builder CDPComponent factory function.
*/
CDPComponent* myCobotLibBuilder::CreateNewComponent(const std::string& type)
{
    
    if (type=="myCobotLib.myCobotIO")
        return new myCobotIO;
    
    return CDPBuilder::CreateNewComponent(type);
}

/**
\internal
Do not edit. Autogenerated Builder CDPBaseObject factory function.
*/
CDPBaseObject* myCobotLibBuilder::CreateNewCDPOperator(const std::string& modelName, const std::string& type, const CDPPropertyBase* inputProperty)
{
  return CDPBuilder::CreateNewCDPOperator(modelName, type, inputProperty);
}

/**
\internal
Do not edit. Autogenerated Builder CDPObject factory function.
*/
CDPObject* myCobotLibBuilder::CreateNewObject(const std::string& type)
{
  return CDPBuilder::CreateNewObject(type);
}

/**
\internal
CDPNode factory function.
*/
CDP::StudioAPI::CDPNode* myCobotLibBuilder::CreateNewCDPNode(const std::string& type)
{
  return CDPBuilder::CreateNewCDPNode(type);
}
