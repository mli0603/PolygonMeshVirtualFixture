#include <mtsDerivedROSBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedROSBridge,
                                      mtsROSBridge,
                                      mtsTaskPeriodicConstructorArg);


mtsDerivedROSBridge::mtsDerivedROSBridge(const std::string &componentName, const double periodInSeconds):
    mtsROSBridge(componentName,periodInSeconds)
{

    std::cout << "constructed from vars" << std::endl;
}

mtsDerivedROSBridge::mtsDerivedROSBridge(const mtsTaskPeriodicConstructorArg &arg):
    mtsROSBridge(arg)
{
    std::cout << "constructed from args" << std::endl;

}

void mtsDerivedROSBridge::Configure(const std::string &filename)
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
        const Json::Value jsonSubscriber = jsonConfig["subscriber"];
        for (unsigned int index = 0; index < jsonSubscriber.size(); ++index) {
            jsonValue = jsonSubscriber[index];
            std::string interfaceName = jsonValue["interface"].asString();
            std::cout << "interface name " << interfaceName << std::endl;

            const Json::Value jsonTopics = jsonValue["topics"];
            for (unsigned int index = 0; index < jsonTopics.size(); ++index) {
                jsonValue = jsonTopics[index];
                std::string cisstType = jsonValue["cisst_type"].asString();
                std::string rosType = jsonValue["ros_type"].asString();
                std::string mtsFunction = jsonValue["mts_function"].asString();
                std::string topicName = jsonValue["topic_name"].asString();

                std::cout << cisstType << rosType << mtsFunction << topicName << std::endl;
                if (cisstType == "bool" && rosType == "std_msgs::Bool"){
                    this->AddSubscriberToCommandWrite<bool,std_msgs::Bool>(interfaceName,
                                                                             mtsFunction,
                                                                             topicName);
                }
            }

            // subscriber has to spin
            mSpin = true;
        }

        const Json::Value jsonPublisher = jsonConfig["publisher"];
        for (unsigned int index = 0; index < jsonPublisher.size(); ++index) {
            jsonValue = jsonPublisher[index];
            std::string interfaceName = jsonValue["interface"].asString();
            std::cout << "interface name " << interfaceName << std::endl;

            const Json::Value jsonTopics = jsonValue["topics"];
            for (unsigned int index = 0; index < jsonTopics.size(); ++index) {
                jsonValue = jsonTopics[index];
                std::string cisstType = jsonValue["cisst_type"].asString();
                std::string rosType = jsonValue["ros_type"].asString();
                std::string mtsFunction = jsonValue["mts_function"].asString();
                std::string topicName = jsonValue["topic_name"].asString();

                std::cout << cisstType << rosType << mtsFunction << topicName << std::endl;

                if (cisstType == "bool" && rosType == "std_msgs::Bool"){
                    this->AddPublisherFromCommandRead<bool,std_msgs::Bool>(interfaceName,
                                                                            mtsFunction,
                                                                            topicName);
                }

            }
        }

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
