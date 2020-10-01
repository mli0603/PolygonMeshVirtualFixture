#ifndef MTSDERIVEDIGTLBRIDGE_H
#define MTSDERIVEDIGTLBRIDGE_H

#include<sawOpenIGTLink/mtsIGTLCRTKBridge.h>

class mtsDerivedIGTLBridge: public mtsIGTLBridge
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
public:
    mtsDerivedIGTLBridge(const std::string & componentName, const double periodInSeconds);
    mtsDerivedIGTLBridge(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedIGTLBridge(){}
protected:
    void Configure(const std::string & filename);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedIGTLBridge);

#endif // MTSDERIVEDIGTLBRIDGE_H
