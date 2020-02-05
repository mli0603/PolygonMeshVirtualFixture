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

#include <mtsDerivedTeleOperationPSM.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

#include <cmath>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const std::string & componentName,
                                                       const double periodInSeconds):
    mtsTeleOperationPSM(componentName, periodInSeconds)
{
}

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTeleOperationPSM(arg)
{
}

// Configure is a virtual method, we can redefine it and have our own
// configuration
void mtsDerivedTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
{
    // Call the base class configure, it will do most of the work
    BaseType::Configure();

    // We want to replace the method called when the MTM is actually
    // driving the PSM
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsDerivedTeleOperationPSM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsDerivedTeleOperationPSM::RunEnabled,
                                this);

    // We need to add a method to retrieve estimated wrench from the
    // PSM
    mtsInterfaceRequired * interfacePSM = GetInterfaceRequired("PSM");
    // That interface should exist, abort otherwise
    CMN_ASSERT(interfacePSM);
    // Add a required function
    interfacePSM->AddFunction("GetWrenchBody",
                              PSMGetWrenchBody);
    interfacePSM->AddFunction("SetWrenchBodyOrientationAbsolute",
                              PSMSetWrenchBodyOrientationAbsolute);
    interfacePSM->AddFunction("GetVelocityCartesian",
                              PSMGetVelocityCartesian);
    interfacePSM->AddFunction("SetConstraintMotionEnable",
                              PSMSetConstraintMotionEnable);

    // Same for MTM
    mtsInterfaceRequired * interfaceMTM = GetInterfaceRequired("MTM");
    CMN_ASSERT(interfaceMTM);
    interfaceMTM->AddFunction("SetWrenchBodyOrientationAbsolute",
                              MTMSetWrenchBodyOrientationAbsolute,
                              MTS_OPTIONAL);

    // ignore jaw
    mIgnoreJaw = true;
    elasticityGain.Assign(1.0,1.0,1.0);
    elasticityGain.Multiply(100.0);
}

void mtsDerivedTeleOperationPSM::EnterEnabled(void)
{
    BaseType::EnterEnabled();
    PSMSetWrenchBodyOrientationAbsolute(true);
    // function SetWrenchBodyOrientationAbsolute is optional on MTM, only call if available
    if (MTMSetWrenchBodyOrientationAbsolute.IsValid()) {
        MTMSetWrenchBodyOrientationAbsolute(true);
    }

    // TODO: call this after registration
    // tell derived psm that constraint motion should be enabled
    PSMSetConstraintMotionEnable(true);
}

void mtsDerivedTeleOperationPSM::RunEnabled(void)
{
    if (mMTM.PositionCartesianCurrent.Valid()
        && mPSM.PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!mIsClutched) {
            // compute mtm Cartesian motion
            vctFrm4x4 mtmPosition(mMTM.PositionCartesianCurrent.Position());

            // translation
            vct3 mtmTranslation;
            vct3 psmTranslation;
            if (mTranslationLocked) {
                psmTranslation = mPSM.CartesianInitial.Translation();
            } else {
                mtmTranslation = (mtmPosition.Translation() - mMTM.CartesianInitial.Translation());
                psmTranslation = mtmTranslation * mScale;
                psmTranslation = mRegistrationRotation * psmTranslation + mPSM.CartesianInitial.Translation();
            }
            // rotation
            vctMatRot3 psmRotation;
            if (mRotationLocked) {
                psmRotation.From(mPSM.CartesianInitial.Rotation());
            } else {
                psmRotation = mRegistrationRotation * mtmPosition.Rotation() * mAlignOffsetInitial;
            }

            // compute desired psm position
            vctFrm4x4 psmCartesianGoal;
            psmCartesianGoal.Translation().Assign(psmTranslation);
            psmCartesianGoal.Rotation().FromNormalized(psmRotation);

            // take into account changes in PSM base frame if any
            if (mBaseFrame.GetPositionCartesian.IsValid()) {
                vctFrm4x4 baseFrame(mBaseFrame.PositionCartesianCurrent.Position());
                vctFrm4x4 baseFrameChange = baseFrame.Inverse() * mBaseFrame.CartesianInitial;
                // update PSM position goal
                psmCartesianGoal = baseFrameChange * psmCartesianGoal;
                // update alignment offset
                mtmPosition.Rotation().ApplyInverseTo(psmCartesianGoal.Rotation(), mAlignOffset);
            }

            // PSM go this cartesian position
            mPSM.PositionCartesianSet.Goal().FromNormalized(psmCartesianGoal);
            mPSM.SetPositionCartesian(mPSM.PositionCartesianSet);

            if (!mIgnoreJaw) {
                // Gripper
                if (mMTM.GetStateGripper.IsValid()) {
                    prmStateJoint gripper;
                    mMTM.GetStateGripper(gripper);
                    mPSM.PositionJointSet.Goal()[0] = GripperToJaw(gripper.Position()[0])
                        + mJawOffset;
                    if (mPSM.PositionJointSet.Goal()[0] < mGripperToJaw.PositionMin) {
                        mPSM.PositionJointSet.Goal()[0] = mGripperToJaw.PositionMin;
                    }
                    mPSM.SetPositionJaw(mPSM.PositionJointSet);
                } else {
                    mPSM.PositionJointSet.Goal()[0] = 45.0 * cmnPI_180;
                    mPSM.SetPositionJaw(mPSM.PositionJointSet);
                }
            }

            // Use proxy location to set haptics feedback; Only if the PSM is following
            if (mIsFollowing){
                prmPositionCartesianGet psmMeasuredCartesian;
                mPSM.GetPositionCartesian(psmMeasuredCartesian);
                vct3 diff = psmCartesianGoal.Translation() - psmMeasuredCartesian.Position().Translation();
                vct3 force;
                for (size_t i=0 ; i < 3; i++){
                    force[i] = - elasticityGain[i] * diff[i];
                }
                if (force.Norm()>1E-3){
                    std::cout << "current haptic feedback is " << force << std::endl;
                }
//                // Re-orient based on rotation between MTM and PSM
//                force = mRegistrationRotation.Inverse() * force;
//                // set force to MTM
//                prmForceCartesianSet wrenchMTM;
//                wrenchMTM.Force().Ref<3>(0) = force;
//                mMTM.SetWrenchBody(wrenchMTM);
            }
            else{
                std::cout << "PSM not following" << std::endl;
            }
        }
    }
}

void mtsDerivedTeleOperationPSM::TransitionEnabled()
{
    std::cout << "Transition Enabled" << std::endl;

    BaseType::TransitionEnabled();

    // tell derived psm that constraint motion should be disabled
    PSMSetConstraintMotionEnable(false);
}
