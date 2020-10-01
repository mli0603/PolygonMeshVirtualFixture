/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Max Zhaoshuo Li
  Created on: 2020-01-24

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

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
