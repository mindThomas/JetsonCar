#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <brisk/brisk.h>
//#include <apriltag/apriltag.h>


class feature_extractor
{

public:
    feature_extractor();
    ~feature_extractor();

    void AprilTagExample(cv::Mat image);
    void ORB_Example(cv::Mat image);
    void BRISK_Example(cv::Mat image);
    void BRISK2_Example(cv::Mat image);

private:
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::BRISK> brisk;
    cv::Ptr<cv::xfeatures2d::SIFT> sift;

    std::shared_ptr<brisk::HarrisScaleSpaceFeatureDetector> briskDetector;
    std::shared_ptr<brisk::BriskDescriptorExtractor> briskExtractor;

    std::shared_ptr<cv::BFMatcher> featureMatcher;

};

#endif // FEATURE_EXTRACTOR_H
