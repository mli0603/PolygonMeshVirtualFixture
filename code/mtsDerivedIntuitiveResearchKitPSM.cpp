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

    // plane constraint
    mPlaneLeft.Name = "PlaneConstraintLeft";
    mPlaneLeft.IneqConstraintRows = 1;
    mPlaneLeft.Normal.Assign(1.0,0.0,0.0);
    mPlaneLeft.PointOnPlane.Assign(0.185, 0.0, 0.0);
    mPlaneLeft.NumJoints = mNumJoints;
    // use the names defined above to relate kinematics data
    mPlaneLeft.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
//    if (!mController->SetVFData(mPlaneLeft))
//    {
//        mController->VFMap.insert(std::pair<std::string, mtsVFPlane*>(mPlaneLeft.Name, new mtsVFPlane(mPlaneLeft.Name, new mtsVFDataPlane(mPlaneLeft))));
//    }

    mPlaneRight.Name = "PlaneConstraintRight";
    mPlaneRight.IneqConstraintRows = 1;
    mPlaneRight.Normal.Assign(-1.0,0.0,0.0);
    mPlaneRight.PointOnPlane.Assign(0.215, 0.0, 0.0);
    mPlaneRight.NumJoints = mNumJoints;
    // use the names defined above to relate kinematics data
    mPlaneRight.KinNames.push_back("MeasuredKinematics"); // need measured kinematics according to mtsVFPlane.cpp
//    if (!mController->SetVFData(mPlaneRight))
//    {
//        mController->VFMap.insert(std::pair<std::string, mtsVFPlane*>(mPlaneRight.Name, new mtsVFPlane(mPlaneRight.Name, new mtsVFDataPlane(mPlaneRight))));
//    }

    // mesh constraint
    if (mMeshFile.LoadMeshFromSTLFile("/home/max/dvrk_ws/src/USAblation/mesh/Skull.STL",true)==-1){
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
            mController->VFMap.insert(std::pair<std::string, mtsVFMesh*>(mMesh.Name, new mtsVFMesh(mMesh.Name, new mtsVFDataMesh(mMesh), mMeshFile)));
        }
    }
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
    std::cout << "Skull to PSM transformation received\n" << transform << std::endl;

    // recompute skull coordinates
    mtsVFMesh* meshConstraint = reinterpret_cast<mtsVFMesh*>(mController->VFMap.find(mMesh.Name)->second);
    meshConstraint->TransformMesh(transform,mMeshFile);

    // recompute plane coordinates
    mPlaneLeft.Normal = transform*mPlaneLeft.Normal;
    mPlaneLeft.PointOnPlane = transform*mPlaneLeft.PointOnPlane;
    mPlaneRight.Normal = transform*mPlaneRight.Normal;
    mPlaneRight.PointOnPlane = transform*mPlaneRight.PointOnPlane;

    // enable constraint motion
    mConstraintMotionEnabled = true;
}
