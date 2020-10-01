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
#include <ros/ros.h>
#include <cisstCommon/cmnGetChar.h>

int main(int argc, char ** argv){
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // component manager
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    // create robot
    simpleTeleop robot("SimpleRobot",20*cmn_ms);

    // add robot to manager
    componentManager->AddComponent(&robot);

    // create ROS node handle
    ros::init(argc, argv, "simpleRobot", ros::init_options::AnonymousName);
    ros::NodeHandle rosNodeHandle;
    // add ros bridge
    mtsROSBridge * subscribers = new mtsROSBridge("subscribers", 0.1 * cmn_ms, &rosNodeHandle);
    subscribers->AddSubscriberToCommandWrite<vctFrm4x4, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
                                                                                    "ServoCartesianPosition",
                                                                                    "/simple_robot/servo_cp");

    componentManager->AddComponent(subscribers);
    mtsROSBridge * publishers = new mtsROSBridge("publishers", 5 * cmn_ms, &rosNodeHandle);
    publishers->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>("RequiresSimpleRobot",
                                                                                                 "GetMeasuredCartesianPosition",
                                                                                                 "/simple_robot/measured_cp");
    componentManager->AddComponent(publishers);

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
    componentManager->AddComponent(igtlBridge);

    // connect componentsS
    componentManager->Connect(subscribers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");
    componentManager->Connect(publishers->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");
    componentManager->Connect(igtlBridge->GetName(), "RequiresSimpleRobot",
                             robot.GetName(), "ProvidesSimpleRobot");

    // create components
    componentManager->CreateAllAndWait(2.0*cmn_s);

    // start periodic run
    componentManager->StartAllAndWait(2.0*cmn_s);

    // ros::spin() callback for subscribers
//    ros::spin();
    do {
        std::cout << "Press 'q' to quit" << std::endl;
    } while (cmnGetChar() != 'q');

    // cleanup
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // stop ROS node
    ros::shutdown();

    return 0;
}
