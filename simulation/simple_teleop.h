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

#ifndef _simpleTeleop_h
#define _simpleTeleop_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>

#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFFollow.h>
#include <sawConstraintController/mtsVFPlane.h>
#include <sawConstraintController/mtsVFLimitsConstraint.h>
#include <sawConstraintController/mtsVFMesh.h>

#include <sawConstraintController/mtsVFDataBase.h>
#include <sawConstraintController/mtsVFDataPlane.h>
#include <sawConstraintController/mtsVFDataJointLimits.h>
#include <sawConstraintController/mtsVFMesh.h>

class simpleTeleop: public mtsTaskPeriodic {
protected:
    // internal method to configure this component
    void init();
    void setupRobot();
    void setupVF();
    nmrConstraintOptimizer::STATUS solve(vctDoubleVec & dq);

    // robot specific variables
    void forwardKinematics(vctDoubleVec& jointPosition);
    vctDoubleVec mJointPosition;
    vctDoubleMat mJacobian;
    vctFrm4x4 mCartesianPosition;
    prmPositionCartesianGet mMeasuredCartesianPosition;// for ros publication

    int mNumDof;
    int mNumJoints;

    // constraint controller
    mtsVFController *mController;
    prmKinematicsState mMeasuredKinematics; // follow crtk convention
    prmKinematicsState mGoalKinematics;
    prmStateJoint mMeasuredJoint;
    mtsVFDataBase mTeleopObjective; // No additional data needed, therefore using mtsVFBase
    mtsVFDataPlane mPlaneConstraint;
    mtsVFDataJointLimits mJointIncrementLimits;
    // mesh
    msh3Mesh mMeshFile;
    mtsVFDataMesh mMesh;
    bool mMeshTransformed;

    void updateOptimizerKinematics();

    // teleop command
    void servoCartesianPosition(const vctFrm4x4 & newGoal);
    void meshFileCallback(const std::string & file_name);
    void SetSkullToPSMTransformIGTL(const prmPositionCartesianSet & transform);
    void servoCartesianTranslation(const vct3 & newGoal);

    vct3 mMeasuredCartesianTranslation;

public:
    // provide a name for the task and define the frequency (time
    // interval between calls to the periodic Run).  Also used to
    // populate the interface(s)
    simpleTeleop(const std::string & componentName, const double periodInSeconds);
    ~simpleTeleop() {}

    // all four methods are pure virtual in mtsTask
    void Run();        // performed periodically
};

#endif // _simpleRobot_h
