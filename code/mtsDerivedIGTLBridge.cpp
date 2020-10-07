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

#include <sawOpenIGTLink/mtsCISSTToIGTL.h>
#include <sawOpenIGTLink/mtsIGTLToCISST.h>
#include <mtsDerivedIGTLBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedIGTLBridge,
                                      mtsIGTLBridge,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedIGTLBridge::mtsDerivedIGTLBridge(const std::string &componentName, const double periodInSeconds):
    mtsIGTLBridge(componentName,periodInSeconds)
{

}

mtsDerivedIGTLBridge::mtsDerivedIGTLBridge(const mtsTaskPeriodicConstructorArg &arg):
    mtsIGTLBridge(arg)
{

}

void mtsDerivedIGTLBridge::Configure(const std::string &filename)
{
    try {
        Json::Value jsonConfig;
        Json::Reader jsonReader;
        std::ifstream jsonStream;
        jsonStream.open(filename.c_str());

        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration file \""
                                     << filename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }


        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;



        Json::Value jsonValue;
        const Json::Value jsonReciever = jsonConfig["receiver"];
        for (unsigned int index = 0; index < jsonReciever.size(); ++index) {
            jsonValue = jsonReciever[index];
            std::string interfaceName = jsonValue["interface"].asString();

            const Json::Value jsonTopics = jsonValue["topics"];
            for (unsigned int index = 0; index < jsonTopics.size(); ++index) {
                jsonValue = jsonTopics[index];
                std::string cisstType = jsonValue["cisst_type"].asString();
                std::string igtlType = jsonValue["igtl_type"].asString();
                std::string mtsFunction = jsonValue["mts_function"].asString();
                std::string topicName = jsonValue["topic_name"].asString();

                if (cisstType == "prmPositionCartesianSet" && igtlType == "igtl::TransformMessage"){
                    this->AddReceiverToCommandWrite<igtl::TransformMessage, prmPositionCartesianSet>(interfaceName,
                                                                                                     mtsFunction,
                                                                                                     topicName);
                }
                if (cisstType == "vct3" && igtlType == "igtl::PointMessage"){
                    this->AddReceiverToCommandWrite<igtl::PointMessage, vct3>(interfaceName,
                                                                              mtsFunction,
                                                                              topicName);
                }
            }
        }

        const Json::Value jsonSender = jsonConfig["sender"];
        for (unsigned int index = 0; index < jsonSender.size(); ++index) {
            jsonValue = jsonSender[index];
            std::string interfaceName = jsonValue["interface"].asString();

            const Json::Value jsonTopics = jsonValue["topics"];
            for (unsigned int index = 0; index < jsonTopics.size(); ++index) {
                jsonValue = jsonTopics[index];
                std::string cisstType = jsonValue["cisst_type"].asString();
                std::string igtlType = jsonValue["igtl_type"].asString();
                std::string mtsFunction = jsonValue["mts_function"].asString();
                std::string topicName = jsonValue["topic_name"].asString();

                if (cisstType == "vct3" && igtlType == "igtl::PointMessage"){
                    this->AddSenderFromCommandRead<vct3, igtl::PointMessage>(interfaceName,
                                                                            mtsFunction,
                                                                            topicName);
                }
            }
        }

        ConfigureJSON(jsonConfig);
        InitServer();

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }
}

