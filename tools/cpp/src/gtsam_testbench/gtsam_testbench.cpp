#include "gtsam_testbench.h"

gtsam_testbench::gtsam_testbench(bool estimate_misalignment, bool estimate_frame_correction)
    : estimate_misalignment_(estimate_misalignment)
    , estimate_frame_correction_(estimate_frame_correction)
{
    //graph = std::make_shared<gtsam::NonlinearFactorGraph>();
    graph = std::make_shared<gtsam::ExpressionFactorGraph>();
    initialValues = std::make_shared<gtsam::Values>();

    imu_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
              deg2rad(1),
              deg2rad(1),
              deg2rad(1)
            ).finished());
    realsense_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
              deg2rad(1),
              deg2rad(1),
              deg2rad(1)
            ).finished());
    extrinsic_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
              deg2rad(0.00001),
              deg2rad(0.00001),
              deg2rad(0.00001)
            ).finished());

    // Define priors of the parameters to estimate and add the to the graph
    gtsam::Rot3 alignmentPrior; // identity/unity prior
    alignmentKey = gtsam::symbol_shorthand::X(0);
    if (estimate_misalignment_) {
        graph->emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(alignmentKey, alignmentPrior,
                                                               gtsam::noiseModel::Diagonal::Sigmas(
                                                                       (gtsam::Vector(3) <<
                                                                           deg2rad(50),
                                                                           deg2rad(50),
                                                                           deg2rad(50)
                                                                       ).finished()));
        initialValues->insert(alignmentKey, alignmentPrior);
    }

    frameCorrectionPrior = gtsam::Rot3::Ry(deg2rad(180));
    frameCorrectionKey = gtsam::symbol_shorthand::X(1);
    if (estimate_frame_correction_) {
        graph->emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(frameCorrectionKey, frameCorrectionPrior,
                                                               gtsam::noiseModel::Diagonal::Sigmas(
                                                                       (gtsam::Vector(3) <<
                                                                               deg2rad(50),
                                                                               deg2rad(50),
                                                                               deg2rad(50)
                                                                       ).finished()));
        initialValues->insert(frameCorrectionKey, frameCorrectionPrior);
    }

    pose_key_index = 0;
}

gtsam_testbench::~gtsam_testbench()
{
}

// Generic compose, assumes existence of traits<T>::Compose
template <typename T>
gtsam::Expression<T> inverse(const gtsam::Expression<T>& t) {
    return gtsam::Expression<T>(gtsam::traits<T>::Inverse, t);
}

inline gtsam::Vector3_ Logmap(const gtsam::Rot3_& R) { return gtsam::Vector3_(&gtsam::Rot3::Logmap, R); }
gtsam::Rot3 transpose_(const gtsam::Rot3& R, gtsam::OptionalJacobian<3,3> H = boost::none) {
    if (H) *H = -R.AdjointMap();
    return R.inverse();
}
inline gtsam::Rot3_ transpose(const gtsam::Rot3_& R) { return gtsam::Rot3_(&transpose_, R); }

//inline gtsam::Rot3_ transpose(const gtsam::Rot3_& R) { return inverse(R); }

gtsam::Rot3_ compose(const gtsam::Rot3_& R1, const gtsam::Rot3& R2) {
    return gtsam::Expression<gtsam::Rot3>(
            [R2](const gtsam::Rot3 &R1, gtsam::OptionalJacobian<3, 3> H) { return R1.compose(R2, H); }, // R1*R2
            R1);
}
gtsam::Rot3_ compose(const gtsam::Rot3& R1, const gtsam::Rot3_& R2) {
    return gtsam::Expression<gtsam::Rot3>(
            [R1](const gtsam::Rot3 &R2, gtsam::OptionalJacobian<3, 3> H) { return R1.compose(R2, H); }, // R1*R2
            R2);
}


void gtsam_testbench::AddMeasurement(const gtsam::Rot3& R_imu_measurement, const gtsam::Rot3& R_realsense_measurement)
{
    //gtsam::Rot3 R_left = gtsam::Rot3::Ry(deg2rad(180));
    //gtsam::Rot3 R_left_transpose = transpose_(R_left);

    gtsam::Key imuKey = gtsam::symbol_shorthand::I(pose_key_index);
    gtsam::Key realsenseKey = gtsam::symbol_shorthand::R(pose_key_index);

    gtsam::Expression<gtsam::Rot3> R_imu(imuKey);
    gtsam::Expression<gtsam::Rot3> R_realsense(realsenseKey);

    // Add measurements as priors
    graph->addExpressionFactor(R_imu, R_imu_measurement, imu_noise);
    graph->addExpressionFactor(R_realsense, R_realsense_measurement, realsense_noise);

    initialValues->insert(imuKey, R_imu_measurement);
    initialValues->insert(realsenseKey, R_realsense_measurement);

    /*
     R_imu_corrected = R_extrinsic * R_frame_correction * R_imu * R_frame_correction'   
     We want to find these by minimizing:   
     R_realsense' * R_imu_corrected = I
     Alternatively two rotation matrices should just be found: 
     R_imu_corrected = R_left * R_imu * R_right
    */
    //gtsam::Rot3_ R_imu_corrected = compose(compose(R_left, R_imu), R_right); // R_left * R_imu * R_right

    // The model was changed to be composed of a frame correction term and a misalignment term:
    // R_imu_corrected = R_alignment * R_frame * R_imu * R_frame'
    //gtsam::Rot3_ R_imu_corrected = compose(compose(compose(R_alignment, R_frame), R_imu), R_frame_transpose); // R_alignment * R_frame * R_imu * R_frame'

    gtsam::Rot3_ R_imu_frame_corrected = R_imu;
    if (estimate_frame_correction_) {
        gtsam::Expression<gtsam::Rot3> R_frame(frameCorrectionKey);
        gtsam::Expression<gtsam::Rot3> R_frame_transpose = gtsam::Expression<gtsam::Rot3>(
                gtsam::traits<gtsam::Rot3>::Inverse, R_frame); //transpose(R_frame);

        R_imu_frame_corrected = compose(compose(R_frame, R_imu), R_frame_transpose); // R_frame * R_imu * R_frame'
    } else {
        R_imu_frame_corrected = compose(compose(frameCorrectionPrior, R_imu), transpose_(frameCorrectionPrior)); // R_frame * R_imu * R_frame'
    }

    gtsam::Rot3_ R_imu_corrected = R_imu_frame_corrected;
    if (estimate_misalignment_) {
        gtsam::Expression<gtsam::Rot3> R_alignment(alignmentKey);

        R_imu_corrected = compose(R_alignment, R_imu_frame_corrected); // R_alignment * R_imu_frame_corrected
    }

    gtsam::Rot3_ R_diff = between(R_realsense, R_imu_corrected); // R_realsense' * R_imu_corrected
    gtsam::Vector3_ err = Logmap(R_diff);

    graph->addExpressionFactor(err, gtsam::Vector3(0,0,0), extrinsic_noise);

    pose_key_index++;
}

gtsam::Values gtsam_testbench::Dogleg_Optimize()
{
    gtsam::Values estimate;

    gtsam::DoglegParams doglegParams;
    gtsam::DoglegOptimizer optimizer(*graph, *initialValues, doglegParams);

    try
    {
        estimate = optimizer.optimize();
    }
    catch(gtsam::IndeterminantLinearSystemException& e)
    {
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter;
        printf("Indeterminate: %s", keyFormatter(e.nearbyVariable()).c_str());
    }

    return estimate;
}

gtsam::Values gtsam_testbench::LM_Optimize()
{
    gtsam::Values estimate;

    gtsam::LevenbergMarquardtParams params;
    params.linearSolverType = gtsam::LevenbergMarquardtParams::MULTIFRONTAL_CHOLESKY;//gtsam::LevenbergMarquardtParams::SEQUENTIAL_CHOLESKY;
    gtsam::LevenbergMarquardtOptimizer optimizer(*graph, *initialValues, params);
    estimate = optimizer.optimize();

    return estimate;
}

gtsam::Values gtsam_testbench::GN_Optimize()
{
    gtsam::Values estimate;

    // Optimize the initial values using a Gauss-Newton nonlinear optimizer
    // The optimizer accepts an optional set of configuration parameters,
    // controlling things like convergence criteria, the type of linear
    // system solver to use, and the amount of information displayed during
    // optimization. We will set a few parameters as a demonstration.
    gtsam::GaussNewtonParams params;
    // Stop iterating once the change in error between steps is less than this value
    params.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    params.maxIterations = 100;

    gtsam::GaussNewtonOptimizer optimizer(*graph, *initialValues, params);
    estimate = optimizer.optimize();

    return estimate;
}

gtsam::Values gtsam_testbench::ISAM2_Optimize()
{
    gtsam::ISAM2Params params;

    params.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
    params.factorization = gtsam::ISAM2Params::CHOLESKY; // CHOLESKY or QR (is slower, but more stable in vision problems)

    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    if (false) {
        params.enableRelinearization = true;
        params.evaluateNonlinearError = false;
    } else {
        params.enableRelinearization = false;
        params.evaluateNonlinearError = true;
    }
    params.enableDetailedResults = true;

    isam = std::make_shared<gtsam::ISAM2>(params);

    gtsam::Values estimate;

    gtsam::ISAM2& isam2 = *isam;
    const gtsam::VariableIndex& index = isam2.getVariableIndex();

    gtsam::Values newValues;
    for (auto it = initialValues->begin(); it != initialValues->end(); it++)
    {
        if (index.find(it->key) == index.end())
        {
            newValues.insert(it->key, it->value);
        }
    }

    try
    {
        isam2.update(*graph, newValues);
        //estimate = isam2.calculateEstimate();
        estimate = isam2.calculateBestEstimate(); // slower but more accurate
        graph->resize(0);
        //marginalCovariance = isam2.marginalCovariance(key);

        try
        {
            //auto jacobian = graph->linearize(isam->getLinearizationPoint())->jacobian();
            //std::cout << jacobian.first << std::endl;
            //Eigen::FullPivLU<gtsam::Matrix> luDecomp(jacobian.first);
            //Eigen::EigenSolver<gtsam::Matrix> eigSolver(jacobian.first);
            //luDecomp.setThreshold(1e-5);
            //std::cerr << "Rank of the jacobian is " << luDecomp.rank() << "/" << jacobian.first.cols() << std::endl;
            //std::cout << (eigSolver.eigenvalues()) << std::endl;
        }
        catch (std::exception& e)
        {
        }
    }
    catch(gtsam::IndeterminantLinearSystemException& e)
    {
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter;
        printf("Indeterminate: %s", keyFormatter(e.nearbyVariable()).c_str());
    }

    return estimate;
}
