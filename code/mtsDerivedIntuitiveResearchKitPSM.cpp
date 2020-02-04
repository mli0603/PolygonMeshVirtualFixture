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
        std::cout << "this is the intial config, skip"<<std::endl;
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

    mJacobianSpatialBase.SetSize(mNumDof,mNumJoints);

    // transformation is [R 0; 0 R]
    mJacobianTransformation.SetSize(mNumDof,mNumDof);
    mJacobianTransformation.Zeros();

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
    mJacobianTransformation.Ref(3,3,0,0).Assign(BaseFrame.Rotation());
    mJacobianTransformation.Ref(3,3,3,3).Assign(BaseFrame.Rotation());
    mJacobianSpatialBase.Assign(mJacobianTransformation * mJacobianSpatial);
    std::cout << "BaseFrm \n" << BaseFrame << std::endl;
    std::cout << "Jacobian Transformation \n " << mJacobianTransformation << std::endl;
    std::cout << "Jacobian spatial local \n" << mJacobianSpatial << std::endl;
    std::cout << "Jacobian spatial in base \n" << mJacobianSpatialBase << std::endl;

    mMeasuredKinematics.Jacobian.Assign(mJacobianSpatialBase); // TODO: check if this jacobain is correct?

    // set goal cartesian position
//    CartesianPositionFrm.From(CartesianSetParam.Goal()); // TODO: recover this
    mGoalKinematics.Frame.Assign(vctFrm3(CartesianPositionFrm));

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

void mtsDerivedIntuitiveResearchKitPSM::ControlPositionCartesian()
{
    if (!mConstraintMotionEnabled){
        BaseType::ControlPositionCartesian();
    }
    else{
        if (mHasNewPIDGoal) {
            // copy current position
            vctDoubleVec jointSet(StateJointKinematics.Position());
            vctDoubleVec jointPosition(jointSet);
            std::cout << "current joint position" << jointSet << std::endl;
            // compute desired arm position
            CartesianPositionFrm.From(CartesianSetParam.Goal());
            if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
                std::cout << "target joint position" << jointSet << std::endl;

                // numerical solve
                vctDoubleVec jointIncrement;
                nmrConstraintOptimizer::STATUS optimizerStatus = Solve(jointIncrement);
                if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
                        std::cout << "incremental joint position" << jointSet-jointPosition << std::endl;
                        std::cout << "linear increment local" << BaseFrame.Rotation().Inverse()*(CartesianGet.Translation()-CartesianPositionFrm.Translation()) << std::endl;
                        std::cout << "linear increment in base" << CartesianGet.Translation()-CartesianPositionFrm.Translation() << std::endl;
                        std::cout << "incremental Cartesian position local using jacobian spatial" << mJacobianSpatial*(jointSet-jointPosition) << std::endl;
                        std::cout << "incremental Cartesian position in base using jacobian spatial" << mJacobianTransformation*mJacobianSpatial*(jointSet-jointPosition) << std::endl;
                        std::cout << "numerical solution" << jointIncrement.Ref(mNumJoints,0) << std::endl<<std::endl;// in case slack is used
                }

                // finally send new joint values
                SetPositionJointLocal(jointSet);
            } else {
                // shows robManipulator error if used
                if (this->Manipulator) {
                    RobotInterface->SendError(this->GetName()
                                              + ": unable to solve inverse kinematics ("
                                              + this->Manipulator->LastError() + ")");
                } else {
                    RobotInterface->SendError(this->GetName() + ": unable to solve inverse kinematics");
                }
            }
            // reset flag
            mHasNewPIDGoal = false;
        }
    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable(const bool &status)
{
    std::cout << "\n\n\nrecived command as "<< status << std::endl;
    mConstraintMotionEnabled = status;
}
