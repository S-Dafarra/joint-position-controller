# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Giulio Romualdi <giulio.romualdi@iit.it>
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

project(joint-controller)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ICUB REQUIRED)

add_subdirectory(Utilities)
add_subdirectory(jointControlModule)

add_subdirectory(app)

include(AddUninstallTarget)
