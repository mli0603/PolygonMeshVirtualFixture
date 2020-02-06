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
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <robHelper.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedIntuitiveResearchKitPSM,
                                      mtsIntuitiveResearchKitPSM,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedIntuitiveResearchKitPSM::mtsDerivedIntuitiveResearchKitPSM(const std::string &componentName,
                                                                     const double periodInSeconds):
    mtsIntuitiveResearchKitPSM(componentName, periodInSeconds)
{

}

mtsDerivedIntuitiveResearchKitPSM::mtsDerivedIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg &arg):
    mtsIntuitiveResearchKitPSM(arg)
{

}

void mtsDerivedIntuitiveResearchKitPSM::Configure(const std::string &filename)
{
    if (filename.empty()){
        // for ros communication
        this->StateTable.AddData(mProxyCartesianPosition, "ProxyCartesianPosition");

        mtsInterfaceProvided * derivedRosInterface = AddInterfaceProvided("providesPSM2");
        if (derivedRosInterface){
            // read constraint motion status
            derivedRosInterface->AddCommandRead(&mtsDerivedIntuitiveResearchKitPSM::ReadConstraintMotionEnable, this, "ReadConstraintMotionEnable");
            // set constraint motion status
            derivedRosInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable, this, "SetConstraintMotionEnable");
            // read proxy location
            derivedRosInterface->AddCommandReadState(this->StateTable, mProxyCartesianPosition, "GetProxyPositionCartesian");
            // set skull to psm transform
            derivedRosInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransform, this, "SetSkullToPSMTransform");
        }
    }
    else{
        BaseType::Configure(filename);

        SetupRobot();
        SetupVF();

        // teleop needs to tell us if constraint motion is enabled or not
        CMN_ASSERT(RobotInterface);
        RobotInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable, this, "SetConstraintMotionEnable");
    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetupVF()
{
    // initialize controller
    mController = new mtsVFController(mNumJoints, mtsVFBase::JPOS);

    // objective
    mTeleopObjective.Name = "Teleop";
    mTeleopObjective.ObjectiveRows = mNumJoints;
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

//    // joint limit constraint
//    mJointLimitsConstraint.Name = "Joint Limit";
//    mJointLimitsConstraint.IneqConstraintRows = 2 * mNumJoints;
//    mJointLimitsConstraint.LowerLimits.SetSize(mNumJoints);
//    mJointLimitsConstraint.LowerLimits.Assign(-0.001, -0.001, -0.001, -0.001, -0.001, -0.001); // TODO: change the velocity limit !!!!!!!!!!!!!!!!!!
//    mJointLimitsConstraint.UpperLimits.SetSize(mNumJoints);
//    mJointLimitsConstraint.UpperLimits.Assign(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
//    mJointLimitsConstraint.KinNames.clear(); // sanity
//    // use the names defined above to relate kinematics data
//    mJointLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp

//    if (!mController->SetVFData(mJointLimitsConstraint))
//    {
//        // Adds a new virtual fixture to the active vector
//        mController->VFMap.insert(std::pair<std::string,mtsVFLimitsConstraint *>(mJointLimitsConstraint.Name,new mtsVFLimitsConstraint(mJointLimitsConstraint.Name,new mtsVFDataJointLimits(mJointLimitsConstraint))));
//    }

    // plane constraint
    mPlaneConstraint.Name = "PlaneConstraint";
    mPlaneConstraint.IneqConstraintRows = 1;
    mPlaneConstraint.Normal.Assign(1.0,0.0,0.0);
    mPlaneConstraint.PointOnPlane.Assign(0.185, 0.0, 0.0);
    mPlaneConstraint.NumJoints = mNumJoints;
    // use the names defined above to relate kinematics data
    mPlaneConstraint.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
    if (!mController->SetVFData(mPlaneConstraint))
    {
        mController->VFMap.insert(std::pair<std::string, mtsVFPlane*>(mPlaneConstraint.Name, new mtsVFPlane(mPlaneConstraint.Name, new mtsVFDataPlane(mPlaneConstraint))));
    }

//    // mesh constraint
//    mMeshFile.LoadMeshFromSTLFile("/home/dvrk-pc/dvrk_ws/src/USAblation/mesh/Cube.STL");
//    mMesh.Name = "Mesh";
//    mMesh.BoudingDistance = 1.0;
//    mMesh.NumJoints = mNumJoints;
//    mMesh.KinNames.clear(); // sanity
//    // use the names defined above to relate kinematics data
//    mMesh.KinNames.push_back("MeasuredKinematics");

//    if (!mController->SetVFData(mMesh))
//    {
//        mController->VFMap.insert(std::pair<std::string, mtsVFMesh*>(mMesh.Name, new mtsVFMesh(mMesh.Name, new mtsVFDataMesh(mMesh), mMeshFile)));
//    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetupRobot()
{
    mNumDof = 6;
    mNumJoints = 6; // jaw does not count

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, mNumJoints);
    mMeasuredKinematics.JointState = &StateJointKinematics; // see mtsIntuitiveResearchKitArm
    // joint already updated by BaseType::GetRobotData() which is called by RunAllState

    mGoalKinematics.Name="GoalKinematics";

    mJacobianBodyBase.SetSize(mNumDof,mNumJoints);

    mConstraintMotionEnabled = false;
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

    // transform jacobian
    Jacobian::ChangeBase(mJacobianBody,CartesianGet,mJacobianBodyBase);
    mMeasuredKinematics.Jacobian.Assign(mJacobianBodyBase);

    // set goal cartesian position
    CartesianPositionFrm.From(CartesianSetParam.Goal()); // compute desired arm position
    mGoalKinematics.Frame.Assign(vctFrm3(CartesianPositionFrm));

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

void mtsDerivedIntuitiveResearchKitPSM::ControlPositionCartesian()
{
    // run without constraint motion
    if (!mConstraintMotionEnabled){
        BaseType::ControlPositionCartesian();
    }
    // constraint motion
    else{
        if (mHasNewPIDGoal) {
            // copy current position
            vctDoubleVec jointPosition(StateJointKinematics.Position());

            // numerical solve
            vctDoubleVec jointIncrement;
            nmrConstraintOptimizer::STATUS optimizerStatus = Solve(jointIncrement);
            if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
                // finally send new joint values
                SetPositionJointLocal(jointPosition+jointIncrement);
            }
            else{
                std::cout << optimizerStatus << std::endl;
                std::cout << "No solution found" << std::endl;
                std::cout << optimizerStatus << std::endl;
            }

//            vctDoubleVec dx(CartesianPositionFrm.Translation()-CartesianGet.Translation());
//            if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
//                if (dx.Norm() > 1E-3){
//                    std::cout << "joint increment \n" << jointSet- jointPosition << std::endl;
//                    if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
//                        std::cout << "numerical solution \n" << jointIncrement.Ref(mNumJoints,0) << std::endl;
//                        std::cout << "error \n" << jointIncrement.Ref(mNumJoints,0) - jointSet + jointPosition  << std::endl<<std::endl;// in case slack is used
//                    }
//                }

            // reset flag
            mHasNewPIDGoal = false;
        }
    }
}

void mtsDerivedIntuitiveResearchKitPSM::GetRobotData()
{
    BaseType::GetRobotData();

    // add proxy position update
    if (mJointReady && mCartesianReady && mHasNewPIDGoal){
        mProxyCartesianPosition.SetTimestamp(StateJointKinematics.Timestamp());
        mProxyCartesianPosition.SetValid(BaseFrameValid);
        mProxyCartesianPosition.Position().From(CartesianSetParam.Goal());
    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable(const bool &status)
{
    mConstraintMotionEnabled = status;
}

void mtsDerivedIntuitiveResearchKitPSM::ReadConstraintMotionEnable(bool &status) const
{
    status = mConstraintMotionEnabled;
}

void mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransform(const vctFrm4x4 &transform)
{
    mSkullToPSMTransform.Assign(transform);

    // TODO: recompute the skull coordinates
    std::cout << "need to recompute skull coordinates" << std::endl;
}
