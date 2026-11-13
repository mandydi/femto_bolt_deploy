#include <industrial_calibration/target_finders/opencv/modified_circle_grid_target_finder.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>

using namespace industrial_calibration;

int main() {
    ModifiedCircleGridTarget target(5, 4, 0.070, true);
    CircleDetectorParams params;
    params.minThreshold = 20;
    params.maxThreshold = 240;
    params.nThresholds = 20;
    params.minRepeatability = 2;
    params.circleInclusionRadius = 5.0;
    params.maxRadiusDiff = 20.0;
    params.filterByColor = true;
    params.circleColor = 255;
    params.filterByArea = true;
    params.minArea = 3000.0;
    params.maxArea = 60000.0;
    params.filterByCircularity = true;
    params.minCircularity = 0.75;

    ModifiedCircleGridTargetFinder finder(target, params);

    // User's new intrinsics
    cv::Mat K = (cv::Mat_<double>(3, 3) << 2247.47, 0, 1872.02, 0, 2246.13, 1090.42, 0, 0, 1);
    cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);

    auto obj_pts_eigen = target.createPoints();
    std::vector<cv::Point3f> obj_pts;
    for (auto const& p : obj_pts_eigen) {
        obj_pts.push_back(cv::Point3f(p.x(), p.y(), p.z()));
    }

    std::cout << "=== Verifying 1105 Left 12 Groups Detection & PnP ===" << std::endl;
    int success_cnt = 0;
    for (int i = 1; i <= 12; ++i) {
        std::string img_path = "/workspace/data/handeye_1105_left12/images/" + std::to_string(i) + ".png";
        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            std::cout << "Frame " << i << ": Failed to load " << img_path << std::endl;
            continue;
        }

        try {
            auto features = finder.findTargetFeatures(img);
            if (features.size() != 20) {
                std::cout << "Frame " << i << ": Found " << features.size() << " features (expected 20)" << std::endl;
                continue;
            }
            std::vector<cv::Point2f> img_pts;
            for (size_t k = 0; k < features.size(); ++k) {
                img_pts.push_back(cv::Point2f(features[k][0].x(), features[k][0].y()));
            }

            cv::Mat rvec, tvec;
            cv::solvePnP(obj_pts, img_pts, K, dist, rvec, tvec);
            double dist_to_cam = cv::norm(tvec);
            std::cout << "Frame " << i << ": SUCCESS (20/20 pts) | PnP Dist=" << dist_to_cam 
                      << " m | tvec=[" << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << "]" << std::endl;
            success_cnt++;
        } catch (const std::exception& e) {
            std::cout << "Frame " << i << ": EXCEPTION: " << e.what() << std::endl;
        }
    }
    std::cout << "Detection summary: " << success_cnt << " / 12 frames detected successfully!" << std::endl;
    return 0;
}
