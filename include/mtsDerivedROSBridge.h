#ifndef MTSDERIVEDROSBRIDGE_H
#define MTSDERIVEDROSBRIDGE_H

#include<cisst_ros_bridge/mtsROSBridge.h>

class mtsDerivedROSBridge: public mtsROSBridge
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    mtsDerivedROSBridge(const std::string & componentName, const double periodInSeconds);
    mtsDerivedROSBridge(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedROSBridge(){}

protected:
    void Configure(const std::string & filename);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedROSBridge);

#endif // MTSDERIVEDROSBRIDGE_H
