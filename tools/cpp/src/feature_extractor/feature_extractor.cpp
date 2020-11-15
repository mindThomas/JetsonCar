#include "feature_extractor.h"

#include <string>
#include <sstream>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <apriltag/common/getopt.h>
}

// See "tools/python/image_processing/feature_extraction_test.py" for more extractor tests using OpenCV

feature_extractor::feature_extractor()
{
    orb = cv::ORB::create(32, 1.0f, 1, 31, 0);
    brisk = cv::BRISK::create();    
#ifndef __ARM_NEON__
    //cv::BRISK::Harris
    briskDetector = std::make_shared<brisk::HarrisScaleSpaceFeatureDetector>(30, 0, 200, 400);
#else
    briskDetector = std::make_shared<brisk::BriskFeatureDetector>(200, 0, true);
#endif
    briskExtractor = std::make_shared<brisk::BriskDescriptorExtractor>(true, true, brisk::BriskDescriptorExtractor::Version::briskV2);

    featureMatcher = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);
}

feature_extractor::~feature_extractor()
{
}

void feature_extractor::AprilTagExample(cv::Mat image)
{
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();
    /*tf = tag25h9_create();
    tf = tag16h5_create();
    tf = tagCircle21h7_create();
    tf = tagCircle49h12_create();
    tf = tagStandard41h12_create();
    tf = tagStandard52h13_create();
    tf = tagCustom48h12_create();*/

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0; // Decimate input image by this factor
    td->quad_sigma = 0.0; // Apply low-pass blur to input
    td->nthreads = 1; // Use this many CPU threads
    td->debug = 1; // Enable debugging output (slow)
    td->refine_edges = 1; // Spend more time trying to align edges of tags

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    std::cout << zarray_size(detections) << " tags detected" << std::endl;

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    line(image, cv::Point(det->p[0][0], det->p[0][1]),
            cv::Point(det->p[1][0], det->p[1][1]),
            cv::Scalar(0, 0xff, 0), 2);
    line(image, cv::Point(det->p[0][0], det->p[0][1]),
            cv::Point(det->p[3][0], det->p[3][1]),
            cv::Scalar(0, 0, 0xff), 2);
    line(image, cv::Point(det->p[1][0], det->p[1][1]),
            cv::Point(det->p[2][0], det->p[2][1]),
            cv::Scalar(0xff, 0, 0), 2);
    line(image, cv::Point(det->p[2][0], det->p[2][1]),
            cv::Point(det->p[3][0], det->p[3][1]),
            cv::Scalar(0xff, 0, 0), 2);

    std::stringstream ss;
    ss << det->id;
    std::string text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                &baseline);
    putText(image, text, cv::Point(det->c[0]-textsize.width/2,
                               det->c[1]+textsize.height/2),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
    apriltag_detections_destroy(detections);

    imshow("Tag Detections", image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

void feature_extractor::ORB_Example(cv::Mat image)
{
    //cv::Mat image = cv::imread(imageFile);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat mask;

    orb->detect(image, keypoints);
    orb->compute(image, keypoints, descriptors);

    cv::Mat out;
    cv::drawKeypoints(image, keypoints, out);

    imshow("ORB Features", out);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void feature_extractor::BRISK_Example(cv::Mat image)
{
    //cv::Mat image = cv::imread(imageFile);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat mask;

    brisk->detect(image, keypoints);
    brisk->compute(image, keypoints, descriptors);

    cv::Mat out;
    cv::drawKeypoints(image, keypoints, out);

    imshow("BRISK Features", out);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void feature_extractor::BRISK2_Example(cv::Mat image)
{
    //cv::Mat image = cv::imread(imageFile);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Mat mask;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    briskDetector->detect(gray, keypoints);
    briskExtractor->compute(gray, keypoints, descriptors);

    cv::Mat out;
    cv::drawKeypoints(image, keypoints, out);

    imshow("BRISK Features", out);
    cv::waitKey(0);
    cv::destroyAllWindows();
}



#if 0
void DetectFeatures()
{
         std::vector<cv::KeyPoint> keypoints;
         cv::Mat descriptors;
         cv::Mat mask;
         printf("Detecting...\n");
         briskDetector->detect(cvImage->image, keypoints);
         briskExtractor->compute(cvImage->image, keypoints, descriptors);
         if (!keypoints.size())
         {
             Timer::Tock("GetNewLandmarks");
             return false;
         }
         printf("Detected %lu keypoints with %d x %d descriptors\n", keypoints.size(), descriptors.rows, descriptors.cols);
}

void ExtractFeatures()
{
         printf("Matching %lu visible landmarks.\n", visibleLandmarks.size());
         std::vector<cv::DMatch> matches;
         std::map<int, float> ratios;
         std::map<int, float> imgDists;


         if (visibleLandmarks.size() > 0)
         {
             cv::Mat visibleDescriptors(cv::Size(descriptors.cols, visibleLandmarks.size()), descriptors.type());
             for (size_t k = 0; k < visibleLandmarks.size(); k++)
             {
                 visibleLandmarks.at(k).descriptor.copyTo(visibleDescriptors.row(k));
                 try
                 {
                     std::pair<gtsam::Point2, bool> projection = cam.projectSafe(visibleLandmarks.at(k).position);
                     if (projection.second && params.drawCandidates)
                     {
                         cv::circle(landmarkDisplay->image, cv::Point2f(projection.first.x(), projection.first.y()), 1, cv::Scalar(0, 255, 255), 1);
                     }

                     projections.push_back(projection);
                 }
                 catch(CheiralityException& e)
                 {
                     projections.push_back(std::make_pair(gtsam::Point2(), false));
                 }
             }

             brisk::BruteForceMatcher bfMatcher;
             std::vector<std::vector<cv::DMatch> > knnMatchesA;
             std::vector<std::vector<cv::DMatch> > knnMatchesB;
             std::vector<std::vector<cv::DMatch> > knnMatches;
             bfMatcher.knnMatch(descriptors, visibleDescriptors, knnMatchesA, 2);
             bfMatcher.knnMatch(visibleDescriptors, descriptors, knnMatchesB, 2);

             for (size_t a = 0; a < knnMatchesA.size(); a++)
             {
                 const std::vector<cv::DMatch>& matchA = knnMatchesA.at(a);
                 if (matchA.size() == 0) continue;

                 for (size_t b = 0; b < knnMatchesB.size(); b++)
                 {
                     if (knnMatchesB.at(b).size() == 0) continue;

                     if (knnMatchesB.at(b).at(0).queryIdx == matchA.at(0).trainIdx &&
                         knnMatchesB.at(b).at(0).trainIdx == matchA.at(0).queryIdx)
                     {
                         knnMatches.push_back(matchA);
                         break;
                     }
                 }
             }

             printf("There are %lu symmetric matches\n", knnMatches.size());

             for (size_t m = 0; m < knnMatches.size(); m++)
             {
                 if (knnMatches[m].size() > 1)
                 {
                     size_t query = knnMatches[m].at(0).queryIdx;
                     size_t train = knnMatches[m].at(0).trainIdx;
                     ratios[query] =  knnMatches[m].at(1).distance / knnMatches[m].at(0).distance;
                     float imgDist = (gtsam::Point2(keypoints.at(query).pt.x, keypoints.at(query).pt.y) - projections.at(train).first).norm();
                     imgDists[query] = imgDist;
                     matches.push_back(knnMatches[m].at(0));
                 }
                 else
                 {
                     ratios[knnMatches[m].at(0).queryIdx]  = (100.0f);
                     imgDists[knnMatches[m].at(0).queryIdx]  = (10000);
                     matches.push_back(knnMatches[m].at(0));
                 }
             }
         }

         printf("Associating %lu matches\n", matches.size());
         std::map<int, int> matchMap;
         std::map<int, int> reverseMatchMap;
         size_t numSuccessfulMatches = 0;
         size_t numRejections = 0;
         for (size_t i = 0; i < matches.size(); i++)
         {
             //printf("%d -> %lu : %f, %f, %f", matches[i].queryIdx, visibleLandmarks[matches[i].trainIdx].id, matches[i].distance, ratios[matches[i].queryIdx], imgDists[matches[i].queryIdx]);
             if    (reverseMatchMap.find(matches[i].trainIdx) == reverseMatchMap.end() &&
                     matches[i].distance < params.hammingThreshold &&
                     ratios[matches[i].queryIdx] > params.ratioThreshold &&
                     imgDists[matches[i].queryIdx] < params.reprojectionThreshold)
             {
                 matchMap[matches[i].queryIdx] = matches[i].trainIdx;
                 reverseMatchMap[matches[i].trainIdx] = matches[i].queryIdx;
                 numSuccessfulMatches++;

                 if (params.drawGoodFeatures)
                 {
                     cv::circle(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, keypoints.at(matches[i].queryIdx).size  * 0.5f, cv::Scalar(0, 255, 0), 2);
                     std::stringstream ss;
                     ss << visibleLandmarks[matches[i].trainIdx].id;
                     cv::putText(landmarkDisplay->image, ss.str(),
                             cv::Point(keypoints.at(matches[i].queryIdx).pt.x + 5, keypoints.at(matches[i].queryIdx).pt.y + keypoints.at(matches[i].queryIdx).size  * 0.5f + 5),
                             cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
                 }


                 if (params.drawGoodMatches)
                 {
                     std::pair<gtsam::Point2, bool> matchProjection = projections.at(matches[i].trainIdx);
                     if (matchProjection.second)
                     {
                         cv::Scalar color = cv::Scalar(0, 255, 0);

                         if (!visibleLandmarks.at(matches[i].trainIdx).isInGraph)
                         {
                             color = cv::Scalar(255, 255, 0);
                         }

                         cv::line(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, cv::Point2f(matchProjection.first.x(), matchProjection.first.y()), color, 1);
                     }
                 }
             }
             else
             {
                 if (params.drawBadFeatures)
                 {
                     cv::circle(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, keypoints.at(matches[i].queryIdx).size * 0.5f, cv::Scalar(0, 0, 255), 1);
                 }

                 if (params.drawBadMatches)
                 {
                     std::pair<gtsam::Point2, bool> matchProjection = projections.at(matches[i].trainIdx);
                     if (matchProjection.second)
                     {
                         cv::line(landmarkDisplay->image, keypoints.at(matches[i].queryIdx).pt, cv::Point2f(matchProjection.first.x(), matchProjection.first.y()), cv::Scalar(0, 0, 255), 1);
                     }
                 }
                 numRejections++;
             }
         }

         printf("%lu successful matches. %lu rejections.\n", numSuccessfulMatches, numRejections);
}
#endif
