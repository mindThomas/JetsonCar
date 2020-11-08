#ifndef GTSAM_IMU_EXTRINSIC_CALIBRATOR_H
#define GTSAM_IMU_EXTRINSIC_CALIBRATOR_H

#include <memory>

//#include "imu/imu.h"
//#include "realsense/realsense.h"
//#include "zed/zed.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Optimizers
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

// Geometry types includes
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot3.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>

// SFM-specific factors
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// Expression includes
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>


#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)


class gtsam_testbench
{

public:
    gtsam_testbench(bool estimate_misalignment = true, bool estimate_frame_correction = true);
	~gtsam_testbench();

	void AddMeasurement(const gtsam::Rot3& R_imu, const gtsam::Rot3& R_realsense);

    gtsam::Values Dogleg_Optimize();
    gtsam::Values LM_Optimize();
    gtsam::Values GN_Optimize();
    gtsam::Values ISAM2_Optimize();

private:
    bool estimate_misalignment_;
    bool estimate_frame_correction_;

    //std::shared_ptr<gtsam::NonlinearFactorGraph> graph;
    std::shared_ptr<gtsam::ExpressionFactorGraph> graph;
    std::shared_ptr<gtsam::Values> initialValues;

    std::shared_ptr<gtsam::ISAM2> isam;

    size_t pose_key_index;
    gtsam::Key alignmentKey;
    gtsam::Key frameCorrectionKey;

    gtsam::Rot3 frameCorrectionPrior;

    gtsam::noiseModel::Diagonal::shared_ptr imu_noise;
    gtsam::noiseModel::Diagonal::shared_ptr realsense_noise;
    gtsam::noiseModel::Diagonal::shared_ptr extrinsic_noise;
};

#endif // GTSAM_IMU_EXTRINSIC_CALIBRATOR_H
