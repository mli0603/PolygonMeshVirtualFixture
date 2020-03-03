/*
  Author(s):  Max Zhaoshuo Li
  Created on: 2019-10-21

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cstdio>
#include "simple_teleop.h"

#include <cisstCommon/cmnConstants.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

simpleTeleop::simpleTeleop(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    init();
}

void simpleTeleop::init() {
    setupRobot();
    setupVF();

    StateTable.AddData(mJacobian,"Jacobian");
    StateTable.AddData(mMeasuredCartesianPosition,"MeasuredCartesianPosition");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("ProvidesSimpleRobot");
    if (interfaceProvided){
        interfaceProvided->AddCommandWrite(&simpleTeleop::servoCartesianPosition, this, "ServoCartesianPosition");
        interfaceProvided->AddCommandReadState(StateTable, mMeasuredCartesianPosition, "GetMeasuredCartesianPosition");
        interfaceProvided->AddCommandWrite(&simpleTeleop::meshFileCallback, this, "MeshFileCallback");
    }
}

void simpleTeleop::setupRobot() {
    mNumDof = 6;
    mNumJoints = 6;

    // joint values
    mJointPosition.SetSize(mNumJoints);
    mJointPosition.SetAll(0.0); // for inverse sphere
    mJointPosition.Ref(3,0).Assign(0.0, 50.0, 0.0).Multiply(cmn_mm); // for normal objects
//    mJointPosition.Ref(3,0).Assign(-30.0, -30.0, -30.0).Multiply(cmn_mm); // for inverse objects
//    mJointPosition.Ref(3,0).Assign(-50.0, -50.0, 35.0).Multiply(cmn_mm); // for inverse pyramid

    // fixed jacobian
    mJacobian.SetSize(mNumDof,mNumJoints);
    mJacobian.SetAll(0.0);
    for (int i = 0; i < mNumJoints; i ++){ // one to one mapping of joint to cartesian position
        mJacobian[i][i] = 1.0;
    }

    // initialize cartesian
    forwardKinematics(mJointPosition);
    mMeasuredCartesianPosition.SetReferenceFrame("map");
    mMeasuredCartesianPosition.Valid() = true;
}

void simpleTeleop::setupVF() {
    // initialize controller
    mController = new mtsVFController(mNumJoints, mtsVFBase::JPOS);

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, mNumJoints);
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredJoint.SetSize(mNumJoints);
    mMeasuredJoint.SetPosition(mJointPosition);
    mMeasuredKinematics.JointState = &mMeasuredJoint;

    mGoalKinematics.Name="GoalKinematics";
    mGoalKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());

    // objective
    mTeleopObjective.Name = "Teleop";
    mTeleopObjective.ObjectiveRows = mNumJoints;
    mTeleopObjective.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mTeleopObjective.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFFollow.cpp
    mTeleopObjective.KinNames.push_back("GoalKinematics"); // goal kinematics needs to be second
    mController->AddVFFollow(mTeleopObjective);

    // joint limit constraint
    mJointIncrementLimits.Name = "Joint Limit";
    mJointIncrementLimits.AbsoluteLimit = false;
    mJointIncrementLimits.IneqConstraintRows = 2 * mNumJoints;
    mJointIncrementLimits.LowerLimits.SetSize(mNumJoints);
    mJointIncrementLimits.LowerLimits.Assign(-0.25, -0.25, -0.25, -1.0, -1.0, -1.0).Multiply(1E-3);
    mJointIncrementLimits.UpperLimits.SetSize(mNumJoints);
    mJointIncrementLimits.UpperLimits.Assign(0.25, 0.25, 0.25, 1.0, 1.0, 1.0).Multiply(1E-3);
    mJointIncrementLimits.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointIncrementLimits.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp
    mController->AddVFLimits(mJointIncrementLimits);

    // mesh constraint
    mMeshFile = cisstMesh(true); // error in mm
    if (mMeshFile.LoadMeshFromSTLFile("/home/max/OneDrive/Research Projects/SickKids/STL/Ascii STL/Concave_Pyramid.stl")==-1){
        CMN_LOG_CLASS_RUN_ERROR << "Cannot load STL file" << std::endl;
        cmnThrow("Cannot load STL file");
    }
    else{
        mMesh.Name = "Mesh";
        mMesh.NumTrianglesInNode = 5;
        mMesh.DiagonalDistanceOfNode = 0.005; // divide a node whenver distance has reached
        mMesh.BoundingDistance = 0.005; // bounding distance for intersection detection
        mMesh.NumJoints = mNumJoints;
        mMesh.KinNames.clear(); // sanity
        // use the names defined above to relate kinematics data
        mMesh.KinNames.push_back("MeasuredKinematics");

        if (!mController->SetVFData(mMesh))
        {
            mController->VFMap.insert(std::pair<std::string, mtsVFMesh*>(mMesh.Name, new mtsVFMesh(mMesh.Name, &mMesh, mMeshFile)));
        }
    }
}

void simpleTeleop::forwardKinematics(vctDoubleVec& jointPosition) {
    mMeasuredCartesianPosition.Position().Translation() = jointPosition.Ref(3,0);
}

void simpleTeleop::Run() {
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // acquire sensor/kinematics information
    // activate/deactivate constraints (or select desired behaviours) if needed

    // solve for next movement
    vctDoubleVec dq;
    nmrConstraintOptimizer::STATUS optimizerStatus =solve(dq);

    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
        mJointPosition += dq.Ref(6,0);
        CMN_LOG_RUN_VERBOSE << 1.0/StateTable.GetAveragePeriod() << std::endl;
        // move
        forwardKinematics(mJointPosition);
    }
    else{
        std::cout << "No solution found" << std::endl;
        std::cout << optimizerStatus << std::endl;

        if (optimizerStatus==nmrConstraintOptimizer::STATUS::NMR_INEQ_CONTRADICTION){
            std::cout << "inqeuality rows " << mController->Optimizer.GetIneqConstraintMatrix().rows() << "cols "<<mController->Optimizer.GetIneqConstraintMatrix().cols()<< std::endl;
            std::cout << "inqeuality vector " << mController->Optimizer.GetIneqConstraintVector().size() << std::endl;
        }

    }
}

void simpleTeleop::updateOptimizerKinematics() {
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(mMeasuredCartesianPosition.Position());
    mMeasuredKinematics.Jacobian.Assign(mJacobian);
    mMeasuredKinematics.JointState->SetPosition(mJointPosition);

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

nmrConstraintOptimizer::STATUS simpleTeleop::solve(vctDoubleVec &dq) {
    // update optimizer kinematics
    updateOptimizerKinematics();

    // update sensor data if needed
    // update vf data if needed

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void simpleTeleop::servoCartesianPosition(const vctFrm4x4 & newGoal) {
    mGoalKinematics.Frame.FromNormalized(newGoal);
}

void simpleTeleop::meshFileCallback(const std::string &file_name)
{
    if (mMeshFile.LoadMeshFromSTLFile(file_name)==-1){
        CMN_LOG_CLASS_RUN_ERROR << "Cannot load STL file" << std::endl;
        cmnThrow("Cannot load STL file");
    }
}
