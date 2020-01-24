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

#include <mtsDerivedIntuitiveResearchKitPSM.h>

mtsDerivedIntuitiveResearchKitPSM::mtsDerivedIntuitiveResearchKitPSM(const std::string &componentName,
                                                                     const double periodInSeconds):
    mtsIntuitiveResearchKitPSM(componentName, periodInSeconds)
{

}

void mtsDerivedIntuitiveResearchKitPSM::SetupVF()
{
    // initialize controller
    mController = new mtsVFController(BaseType::NumberOfJoints(), mtsVFBase::JPOS);

    // objective
    mTeleopObjective.Name = "Teleop";
    mTeleopObjective.ObjectiveRows = NumberOfJoints();
    mTeleopObjective.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mTeleopObjective.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFFollow.cpp
    mTeleopObjective.KinNames.push_back("GoalKinematics"); // goal kinematics needs to be second

    // add objective and constraint to optimizer
    // first, we check if we can set the data. If not, we insert it.
    if (!mController->SetVFData(mTeleopObjective))
    {
        // Adds a new virtual fixture to the active vector
        mController->VFMap.insert(std::pair<std::string,mtsVFFollow *>(mTeleopObjective.Name,new mtsVFFollow(mTeleopObjective.Name,new mtsVFDataBase(mTeleopObjective))));
    }

    // joint limit constraint
    mJointLimitsConstraint.Name = "Joint Limit";
    mJointLimitsConstraint.IneqConstraintRows = 2 * NumberOfJoints();
    mJointLimitsConstraint.LowerLimits.SetSize(NumberOfJoints());
    mJointLimitsConstraint.LowerLimits.Assign(-0.001, -0.001, -0.001, -0.001, -0.001, -0.001); // TODO: change the velocity limit !!!!!!!!!!!!!!!!!!
    mJointLimitsConstraint.UpperLimits.SetSize(NumberOfJoints());
    mJointLimitsConstraint.UpperLimits.Assign(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
    mJointLimitsConstraint.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp

    if (!mController->SetVFData(mJointLimitsConstraint))
    {
        // Adds a new virtual fixture to the active vector
        mController->VFMap.insert(std::pair<std::string,mtsVFLimitsConstraint *>(mJointLimitsConstraint.Name,new mtsVFLimitsConstraint(mJointLimitsConstraint.Name,new mtsVFDataJointLimits(mJointLimitsConstraint))));
    }


    // mesh constraint
    mMeshFile.LoadMeshFromSTLFile("/home/dvrk-pc/dvrk_ws/src/USAblation/mesh/Cube.STL");
    mMesh.Name = "Mesh";
    mMesh.BoudingDistance = 1.0;
    mMesh.NumJoints = NumberOfJoints();
    mMesh.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mMesh.KinNames.push_back("MeasuredKinematics");

    if (!mController->SetVFData(mMesh))
    {
        mController->VFMap.insert(std::pair<std::string, mtsVFMesh*>(mMesh.Name, new mtsVFMesh(mMesh.Name, new mtsVFDataMesh(mMesh), mMeshFile)));
    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetupRobot()
{
    mNumDof = 6;

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, NumberOfJoints());
    mMeasuredKinematics.JointState = &StateJointPID; // see mtsIntuitiveResearchKitArm

    mGoalKinematics.Name="GoalKinematics";
}

nmrConstraintOptimizer::STATUS mtsDerivedIntuitiveResearchKitPSM::Solve(vctDoubleVec &dq)
{
    // update kinematics parameters
    UpdateOptimizerKinematics();

    // update optimizer
    mController->UpdateOptimizer(StateTable.GetAveragePeriod());

    // solve
    return mController->Solve(dq);
}

void mtsDerivedIntuitiveResearchKitPSM::UpdateOptimizerKinematics()
{
    // update cartesian position and jacobian
    mMeasuredKinematics.Frame.FromNormalized(CartesianGet); // see mtsIntuitiveResearchKitArm
    mMeasuredKinematics.Jacobian.Assign(mJacobianBody); // TODO: check if body should be used

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

void mtsDerivedIntuitiveResearchKitPSM::Configure(const std::string & filename)
{
    BaseType::Configure(filename);

    SetupVF();
    SetupRobot();
}
