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

#include "simple_teleop.h"
#include <cstdio>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawOpenIGTLink/mtsCISSTToIGTL.h>
#include <sawOpenIGTLink/mtsIGTLToCISST.h>
#include <sawOpenIGTLink/mtsIGTLBridge.h>

int main(){
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // component manager
    mtsComponentManager * componentManger = mtsComponentManager::GetInstance();

    // create robot
    simpleTeleop robot("SimpleRobot",20*cmn_ms);

    // add robot to manager
    componentManger->AddComponent(&robot);

    // add ros bridge
    mtsROSBridge * subscribers = new mtsROSBridge("subscribers", 0.1 * cmn_ms, true /* spin */);
    subscribers->AddSubscriberToCommandWrite<vctFrm4x4, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
                                                                                    "ServoCartesianPosition",
                                                                                    "/simple_robot/servo_cp");

    componentManger->AddComponent(subscribers);
    mtsROSBridge * publishers = new mtsROSBridge("publishers", 5 * cmn_ms);
    publishers->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
                                                                                                 "GetMeasuredCartesianPosition",
                                                                                                 "/simple_robot/measured_cp");
    componentManger->AddComponent(publishers);

    // add openIgtlink
    mtsIGTLBridge * igtlBridge = new mtsIGTLBridge("simpleTelop-igtl", 5 * cmn_ms);
    igtlBridge->AddSenderFromCommandRead<vct3, igtl::PointMessage>("RequiresSimpleRobot",
                                                                   "GetMeasuredCartesianTranslation",
                                                                   "measured_cp");
    igtlBridge->AddReceiverToCommandWrite<igtl::PointMessage, vct3>("RequiresSimpleRobot",
                                                                    "SetServoCartesianTranslation",
                                                                    "servo_cp");
    igtlBridge->AddReceiverToCommandWrite<igtl::TransformMessage, prmPositionCartesianSet>("RequiresSimpleRobot",
                                                                                           "SetSkullToPSMTransformIGTL",
                                                                                           "Skull To PSM");

    igtlBridge->SetPort(18944);
    componentManger->AddComponent(igtlBridge);

    // connect componentsS
    componentManger->Connect(subscribers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");
    componentManger->Connect(publishers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");
    componentManger->Connect(igtlBridge->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");

    // create components
    componentManger->CreateAll();
    componentManger->WaitForStateAll(mtsComponentState::READY, 2.0*cmn_s);

    // start periodic run
    componentManger->StartAll();
    componentManger->WaitForStateAll(mtsComponentState::ACTIVE, 2.0*cmn_s);

    // ros::spin() callback for subscribers
    ros::spin();

    // cleanup
    componentManger->KillAll();
    componentManger->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManger->Cleanup();

    return 0;
}
