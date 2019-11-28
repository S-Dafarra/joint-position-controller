/**
 * @file WalkingModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef JOINT_CONTROL_HPP
#define JOINT_CONTROL_HPP

// std
#include <memory>
#include <deque>
#include <mutex>

// YARP
#include <yarp/os/RFModule.h>

// iDynTree
#include <iDynTree/Core/VectorDynSize.h>

#include <RobotHelper.hpp>

/**
 * RFModule of the Walking controller
 */
class JointControlModule: public yarp::os::RFModule
{
    double m_dT; /**< RFModule period. */
    std::string m_robot; /**< Robot name. */
    std::mutex m_mutex;
    bool m_running = false;


    std::unique_ptr<RobotHelper> m_robotControlHelper; /**< Robot control helper. */

    std::deque<iDynTree::VectorDynSize> m_qDesired; /**< Vector containing the results of the IK alg

    /**
     * Advance the reference signal.
     * @return true in case of success and false otherwise.
     */
    bool advanceReferenceSignals();

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

};
#endif
