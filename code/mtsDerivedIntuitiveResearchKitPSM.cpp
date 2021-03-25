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
        // for igtl communication
        this->StateTable.AddData(mMeasuredCartesianTranslation, "MeasuredCartesianTranslation");
        this->StateTable.AddData(mProxyCaertesianTranslation, "ProxyCaertesianTranslation");

        mtsInterfaceProvided * bridgeInterface = AddInterfaceProvided("providesPSM2");
        if (bridgeInterface){
            // read constraint motion status
            bridgeInterface->AddCommandRead(&mtsDerivedIntuitiveResearchKitPSM::ReadConstraintMotionEnable, this, "ReadConstraintMotionEnable");
            // set constraint motion status
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable, this, "SetConstraintMotionEnable");
            // set simulation status
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetSimulation, this, "SetSimulation");
            // read proxy location
            bridgeInterface->AddCommandReadState(this->StateTable, mProxyCartesianPosition, "GetProxyPositionCartesian");
            // set skull to psm transform
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransform, this, "SetSkullToPSMTransform");
            // set mesh constraint enable
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetMeshConstraintEnable, this, "SetMeshConstraintEnable");

            bridgeInterface->AddCommandReadState(this->StateTable, mMeasuredCartesianTranslation, "GetMeasuredCartesianTranslation");
            bridgeInterface->AddCommandReadState(this->StateTable, mProxyCaertesianTranslation, "GetServoCartesianTranslation");
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransformIGTL, this, "SetSkullToPSMTransformIGTL");
            bridgeInterface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::DummyReceiver, this, "DummyReceiver");
        }
    }
    else{
        BaseType::Configure(filename);

        SetupRobot();
        SetupVF();

        // teleop needs to tell us if constraint motion is enabled or not
        CMN_ASSERT(m_arm_interface);
        m_arm_interface->AddCommandWrite(&mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable, this, "SetConstraintMotionEnable");
        m_arm_interface->AddCommandRead(&mtsDerivedIntuitiveResearchKitPSM::GetSimulation, this, "GetSimulation");
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
    mController->AddVFFollow(mTeleopObjective);

    // joint limit constraint
    mJointIncLimitsConstraint.Name = "Joint Increment Limit";
    mJointIncLimitsConstraint.IneqConstraintRows = 2 * mNumJoints;
    mJointIncLimitsConstraint.AbsoluteLimit = false;
    mJointIncLimitsConstraint.UpperLimits.SetSize(mNumJoints);
    mJointIncLimitsConstraint.UpperLimits.Assign(0.019,0.027,0.00072,0.085,0.075,0.082);
    mJointIncLimitsConstraint.LowerLimits.SetSize(mNumJoints);
    mJointIncLimitsConstraint.LowerLimits.Assign(mJointIncLimitsConstraint.UpperLimits).Multiply(-1.0);
    mJointIncLimitsConstraint.KinNames.clear(); // sanity
    // use the names defined above to relate kinematics data
    mJointIncLimitsConstraint.KinNames.push_back("MeasuredKinematics"); // measured kinematics needs to be first according to mtsVFLimitsConstraint.cpp
    mController->AddVFLimits(mJointIncLimitsConstraint);


    /****** The following are defined in the skull coordinate frame, which will be transformed into PSM coordinate frame *******/
    // mesh constraint
    mMeshFile = msh3Mesh(0.5,true); // 0.5 mm error for PSM
    std::string filePath = __FILE__;
    filePath.erase(filePath.end()-42,filePath.end());
    filePath.append("mesh/Skull.stl");
    std::cout << filePath << std::endl;
    if (mMeshFile.LoadMeshFromSTLFile(filePath)==-1){
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

void mtsDerivedIntuitiveResearchKitPSM::SetupRobot()
{
    mNumDof = 6;
    mNumJoints = 6; // jaw does not count

    // robot related data
    mMeasuredKinematics.Name="MeasuredKinematics";
    mMeasuredKinematics.Jacobian.SetSize(mNumDof, mNumJoints);
    mMeasuredKinematics.JointState = &m_kin_measured_js; // see mtsIntuitiveResearchKitArm
    // joint already updated by BaseType::GetRobotData() which is called by RunAllState

    mGoalKinematics.Name="GoalKinematics";

    mJacobianBodyBase.SetSize(mNumDof,mNumJoints);
    mJacobianBodyBaseInverse.SetSize(mNumJoints,mNumDof);

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
    mMeasuredKinematics.Frame.FromNormalized(m_measured_cp_frame); // see mtsIntuitiveResearchKitArm

    // transform jacobian
    Jacobian::ChangeBase(m_body_jacobian,m_measured_cp_frame,mJacobianBodyBase);
    mMeasuredKinematics.Jacobian.Assign(mJacobianBodyBase);

    // set goal cartesian position
    CartesianPositionFrm.From(CartesianSetParam.Goal()); // compute desired arm position
    mGoalKinematics.Frame.Assign(vctFrm3(CartesianPositionFrm));

    // update controller stored kinematics
    mController->SetKinematics(mMeasuredKinematics);
    mController->SetKinematics(mGoalKinematics);
}

void mtsDerivedIntuitiveResearchKitPSM::control_servo_cp()
{
    // run without constraint motion
    if (!mConstraintMotionEnabled){
        BaseType::control_servo_cp();
    }
    // constraint motion
    else{
        if (m_new_pid_goal) {
            // copy current position
            vctDoubleVec jointPosition(m_kin_measured_js.Position());

            // numerical solve
            vctDoubleVec jointInc, jointSlack;
            nmrConstraintOptimizer::STATUS optimizerStatus = Solve(jointInc);
            if (optimizerStatus == nmrConstraintOptimizer::STATUS::NMR_OK){
                // finally send new joint values
                servo_jp_internal(jointPosition+jointInc.Ref(mNumDof,0));
            }
            else{
                CMN_LOG_CLASS_RUN_ERROR << "Constraint optimization error: No solution found" << std::endl;
                CMN_LOG_CLASS_RUN_ERROR << optimizerStatus << std::endl;
            }

            // reset flag
            m_new_pid_goal = false;
        }
    }
}

void mtsDerivedIntuitiveResearchKitPSM::GetRobotData()
{
    BaseType::GetRobotData();

    // add proxy position update
    if (IsJointReady() && IsCartesianReady() && m_new_pid_goal){
        mProxyCartesianPosition.SetTimestamp(m_kin_measured_js.Timestamp());
        mProxyCartesianPosition.SetValid(m_base_frame_valid);
        mProxyCartesianPosition.Position().From(CartesianSetParam.Goal());

        mProxyCaertesianTranslation.Assign(mProxyCartesianPosition.Position().Translation()).Multiply(cmn_m/cmn_mm);
    }

    mMeasuredCartesianTranslation.Assign(m_measured_cp_frame.Translation()).Multiply(cmn_m/cmn_mm);
}

void mtsDerivedIntuitiveResearchKitPSM::SetConstraintMotionEnable(const bool &status)
{
    mConstraintMotionEnabled = status;
}

void mtsDerivedIntuitiveResearchKitPSM::ReadConstraintMotionEnable(bool &status) const
{
    status = mConstraintMotionEnabled;
}

void mtsDerivedIntuitiveResearchKitPSM::SetMeshConstraintEnable(const bool &status)
{
    mMesh.Active = status;
    std::cout << "Mesh constraint state is " << mMesh.Active << std::endl;
}

void mtsDerivedIntuitiveResearchKitPSM::SetSimulation(const bool &status)
{
    mSimulated = status;
}

void mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransform(const vctFrm4x4 &transform)
{
    if (!mConstraintMotionEnabled && (m_powered || mSimulated)){ // register when the constraint motion is not enabled (prevent receiving multiple times) and when the arm is powered
        std::cout << "Skull to PSM transformation received\n" << transform << std::endl;

        // recompute skull coordinates
        mtsVFMesh* meshConstraint = reinterpret_cast<mtsVFMesh*>(mController->VFMap.find(mMesh.Name)->second);
        meshConstraint->TransformMesh(transform,mMeshFile);

        // enable constraint motion
        mConstraintMotionEnabled = true;
    }
    else{
        if (mConstraintMotionEnabled){
            CMN_LOG_CLASS_RUN_ERROR << "Constraint motion already enabled, reset the flag to receive new registration" << std::endl;
        }
        if (!m_powered){
            CMN_LOG_CLASS_RUN_ERROR << "Arm is not powered" << std::endl;
        }

    }
}

void mtsDerivedIntuitiveResearchKitPSM::SetSkullToPSMTransformIGTL(const prmPositionCartesianSet &transform)
{
    if (!mConstraintMotionEnabled && (m_powered || mSimulated)){ // register when the constraint motion is not enabled (prevent receiving multiple times) and when the arm is powered
        std::cout << "Skull to PSM transformation received\n" << transform << std::endl;

        // recompute skull coordinates
        vctFrm4x4 f;
        f.From(transform.Goal());
        f.Translation().Multiply(cmn_mm/cmn_m);
        mtsVFMesh* meshConstraint = reinterpret_cast<mtsVFMesh*>(mController->VFMap.find(mMesh.Name)->second);
        meshConstraint->TransformMesh(f,mMeshFile);

        // enable constraint motion
        mConstraintMotionEnabled = true;
    }
    else{
        if (mConstraintMotionEnabled){
            CMN_LOG_CLASS_RUN_ERROR << "Constraint motion already enabled, reset the flag to receive new registration" << std::endl;
        }
        if (!m_powered){
            CMN_LOG_CLASS_RUN_ERROR << "Arm is not powered" << std::endl;
        }

    }
}

void mtsDerivedIntuitiveResearchKitPSM::GetSimulation(bool &status) const
{
    status = mSimulated;
}

void mtsDerivedIntuitiveResearchKitPSM::DummyReceiver(const vct3 &dummy)
{

}
