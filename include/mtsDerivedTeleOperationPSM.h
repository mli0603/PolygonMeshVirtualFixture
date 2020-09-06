/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Max Li
  Created on: 2020-01-24

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDerivedTeleOperationPSM_h
#define _mtsDerivedTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

class mtsDerivedTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    typedef mtsTeleOperationPSM BaseType;

    mtsDerivedTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedTeleOperationPSM(){}

    void Configure(const std::string & CMN_UNUSED(filename));

protected:
    void EnterEnabled(void);
    void RunEnabled(void);
    void TransitionEnabled(void);
    
    mtsFunctionRead  PSMGetWrenchBody;
    mtsFunctionWrite PSMSetWrenchBodyOrientationAbsolute;
    mtsFunctionRead  PSMGetVelocityCartesian;
    mtsFunctionWrite MTMSetWrenchBodyOrientationAbsolute;
    mtsFunctionWrite PSMSetConstraintMotionEnable;
    mtsFunctionRead  PSMGetSimulation;

    vctFrm4x4 mPSMProxyMeasuredCartesianPosition;
    vct3 elasticityGain;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedTeleOperationPSM);

#endif // _mtsDerivedTeleOperationPSM_h
