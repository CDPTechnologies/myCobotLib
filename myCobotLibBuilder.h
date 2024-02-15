/**
myCobotLibBuilder header file.
*/

#ifndef MYCOBOTLIB_MYCOBOTLIBBUILDER_H
#define MYCOBOTLIB_MYCOBOTLIBBUILDER_H

#include <CDPSystem/Application/CDPBuilder.h>

namespace myCobotLib {

class myCobotLibBuilder : public CDPBuilder
{
public:
  myCobotLibBuilder(const char* libName);
  CDPComponent* CreateNewComponent(const std::string& type) override;
  CDPBaseObject* CreateNewCDPOperator(const std::string& modelName,const std::string& type,const CDPPropertyBase* inputProperty) override;
  CDPObject* CreateNewObject(const std::string& type) override;
  CDP::StudioAPI::CDPNode* CreateNewCDPNode(const std::string& type) override;
};

}

#endif
