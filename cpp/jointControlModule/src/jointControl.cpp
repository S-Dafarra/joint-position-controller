/**
 * @file WalkingModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>
#include <memory>
#include <fstream>
#include <chrono>
#include <thread>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>

#include <jointControl.hpp>
#include <Utils.hpp>
#include <FolderPath.h>

std::pair<bool, std::deque<iDynTree::VectorDynSize>> readStateFromFile(const std::string& filename, const std::size_t num_fields)
{
    std::deque<iDynTree::VectorDynSize> data;

    yInfo() << "Opening " << filename;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << "Failed to open " << filename << '\n';
        return std::make_pair(false, data);
    }
    else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        iDynTree::VectorDynSize vector;
        vector.resize(static_cast<unsigned int>(num_fields));
        std::size_t found_lines = 0;
        for (auto& l : istrm_strings)
        {
            unsigned int found_fields = 0;
            std::string number_str;
            std::istringstream iss(l);

            while (iss >> number_str)
            {
                vector(found_fields) = std::stod(number_str);
                found_fields++;
            }
            if (num_fields != found_fields)
            {
                std::cout << "Malformed input file " << filename << '\n';

                return std::make_pair(false, data);
            }
            data.push_back(vector);
            found_lines++;
        }

        return std::make_pair(true, data);
    }
}


bool JointControlModule::advanceReferenceSignals()
{
    // check if vector is not initialized
    if(m_qDesired.empty())
    {
        yError() << "[jointControlModule::advanceReferenceSignals] Cannot advance empty reference signals.";
        return false;
    }

    m_qDesired.pop_front();
    m_qDesired.push_back(m_qDesired.back());

    return true;
}

double JointControlModule::getPeriod()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    //  period of the module (seconds)
    return m_dT;
}

bool JointControlModule::configure(yarp::os::ResourceFinder& rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.01)).asDouble();
    std::string name;
    if(!YarpHelper::getStringFromSearchable(generalOptions, "name", name))
    {
        yError() << "[JointControlModule::configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());

    std::string datasetType;
    if(!YarpHelper::getStringFromSearchable(generalOptions, "datasetType", datasetType))
    {
        yError() << "[JointControlModule::configure] Unable to get the string \"datasetType\" from searchable.";
        return false;
    }

    m_robotControlHelper = std::make_unique<RobotHelper>();
    yarp::os::Bottle& robotControlHelperOptions = rf.findGroup("ROBOT_CONTROL");
    robotControlHelperOptions.append(generalOptions);
    if(!m_robotControlHelper->configureRobot(robotControlHelperOptions))
    {
        yError() << "[JointControlModule::configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle& forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    forceTorqueSensorsOptions.append(generalOptions);
    if(!m_robotControlHelper->configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[JointControlModule::configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    yarp::os::Bottle& pidOptions = rf.findGroup("PID");
    if (!m_robotControlHelper->configurePIDHandler(pidOptions))
    {
        yError() << "[JointControlModule::configure] Failed to configure the PIDs.";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));


    m_qDesired.clear();

    auto data = readStateFromFile(getAbsDirPath("txtDatasets/" + datasetType + "/jointDataset.txt"), 23);
    if(!data.first)
    {
        return false;
    }

    m_qDesired = data.second;

    if(!m_robotControlHelper->setPositionReferences(m_qDesired.front(), 5.0))
    {
        yError() << "[JointControlModule::configure] Error while setting the initial position.";
        return false;
    }

    bool motionDone = false;
    while (!motionDone)
    {
        if(!m_robotControlHelper->checkMotionDone(motionDone))
        {
            yError() << "[WalkingModule::updateModule] Unable to check if the motion is done";
            return false;
        }
    }

    yInfo() << "[JointControlModule::configure] Preparation completed. Switching to position direct.";

    if(!m_robotControlHelper->switchToControlMode(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "Failed to switchtocontrolmode";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    yInfo() << "[JointControlModule::configure] Ready to play!";

    return true;
}


bool JointControlModule::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // restore PID
    m_robotControlHelper->getPIDHandler().restorePIDs();

    // close the connection with robot
    if(!m_robotControlHelper->close())
    {
        yError() << "[JointControlModule::close] Unable to close the connection with the robot.";
        return false;
    }

    return true;
}

bool JointControlModule::updateModule()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_running) {
        if ((m_robotControlHelper->getLeftWrench().getLinearVec3()(2) > 100) && (m_robotControlHelper->getRightWrench().getLinearVec3()(2) > 100))
        {
            for (int i = 0; i < 5; ++i)
            {
                yInfo() << "Starting...." << (5-i);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            m_running = true;
        }
        return true;
    }

    if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired.front()))
    {
        yError() << "[JointControlModule::updateModule] Error while setting the reference position to iCub.";
        return false;
    }

    advanceReferenceSignals();
    return true;
}
