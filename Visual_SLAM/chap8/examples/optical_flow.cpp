/*
 * @Description: 光流法
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-04 21:03:51
 */


#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


class OpticalFlowTracker
{
public:
    OpticalFlowTracker(
            const cv::Mat & img1,
            const cv::Mat & img2,
            const std::vector<cv::KeyPoint> & kp1,
            std::vector<cv::KeyPoint> & kp2,
            std::vector<bool> & success,
            bool inverse = true,
            bool has_initial = false) :
            img1(img1), img2(img2), kp1(kp1), kp2(kp2), success(success), inverse(inverse), has_initial(has_initial)
            {}

    void calculateOpticalFlow(const cv::Range & range);

private:
    const cv::Mat & img1;
    const cv::Mat & img2;
    const std::vector<cv::KeyPoint> & kp1;
    std::vector<cv::KeyPoint> & kp2;
    std::vector<bool> & success;
    bool inverse = true;
    bool has_initial = false;
};


void opticalFlowSingleLevel(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const std::vector<cv::KeyPoint> & kp1,
        std::vector<cv::KeyPoint> & kp2,
        std::vector<bool> & success,
        bool inverse = false,
        bool has_initial = false
        );


void opticalFlowMultiLevel(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const std::vector<cv::KeyPoint> & kp1,
        std::vector<cv::KeyPoint> & kp2,
        std::vector<bool> & success,
        bool inverse = false
        );


/**
 * get a gray scale value by bi-linear interpolation
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of the pixel
 */
inline float getPixelValue(const cv::Mat & img, float x, float y)
{
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols - 1)
        x = img.cols - 2;
    if (y >= img.rows - 1)
        y = img.rows - 2;

    float xx = x - floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x) + 1);
    int y_a1 = std::min(img.rows - 1, int(y) + 1);

    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x)
           + xx * (1 - yy) * img.at<uchar>(y, x_a1)
           + (1 - xx) * yy * img.at<uchar>(y_a1, x)
           + xx * yy * img.at<uchar>(y_a1, x_a1);
}


int main()
{
    cv::Mat img1 = cv::imread("../LK1.png");
    cv::Mat img2 = cv::imread("../LK2.png");

    // key points in image1
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(500, 0.01, 20);
    detector->detect(img1, kp1);

    // single level LK
    std::vector<cv::KeyPoint> kp2_single;
    std::vector<bool> success_single;
    opticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

    // multi level LK
    std::vector<cv::KeyPoint> kp2_multi;
    std::vector<bool> success_multi;
    opticalFlowSingleLevel(img1, img2, kp1, kp2_multi, success_multi);

    // use Opencv to validate
    std::vector<cv::Point2f> pt1, pt2;
    for (auto kp : kp2_single)
        pt1.push_back(kp.pt);

    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);

    // plot difference
    cv::Mat img2_single;
    cv::cvtColor(img2, img2_single, cv::COLOR_BGR2GRAY);
    for (int i = 0; i < kp2_single.size(); ++i)
    {
        if (success_single[i])
        {
            cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 255, 0), 2);
            cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 255, 0));
        }
    }

    cv::Mat img2_multi;
    cv::cvtColor(img2, img2_multi, cv::COLOR_BGR2GRAY);
    for (int i = 0; i < kp2_multi.size(); ++i)
    {
        if (success_multi[i])
        {
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 255, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 255, 0));
        }
    }

    cv::Mat img2_cv;
    cv::cvtColor(img2, img2_cv, cv::COLOR_BGR2GRAY);
    for (int i = 0; i < pt2.size(); ++i)
    {
        if (status[i])
        {
            cv::circle(img2_cv, pt2[i], 2, cv::Scalar(0, 255, 0), 2);
            cv::line(img2_cv, pt1[i], pt2[i], cv::Scalar(0, 255, 0));
        }
    }

    cv::imshow("tracked by single level", img2_single);
    cv::imshow("tracked by multi level", img2_multi);
    cv::imshow("tracked by cv", img2_cv);
    cv::waitKey(0);

    return 0;
}


void OpticalFlowTracker::calculateOpticalFlow(const cv::Range & range)
{
    int half_patch_size = 4;
    int iterations = 10;

    for (int i = range.start; i < range.end; ++i)
    {
        double dx = 0, dy = 0;  // dx/dt, dy/dt, need to be estimated
        if (has_initial)
        {
            dx = kp2[i].pt.x - kp1[i].pt.x;
            dy = kp2[i].pt.y - kp1[i].pt.y;
        }

        double cost, last_cost = 0;
        bool succeed;   // indicate if this point succeeded

        for (int iter = 0; iter < iterations; ++iter)
        {
            Eigen::Matrix2d H;  // hessian
            if (!inverse || iter == 0)  // 反向光流法中H只需要计算一次
                H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();  // bias
            Eigen::Vector2d J;  // jacobian

            cost = 0;

            for (int x = -half_patch_size; x < half_patch_size; ++x)
                for (int y = -half_patch_size; y < half_patch_size; ++y)
                {
                    auto kp = kp1[i];
                    double error = getPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                   getPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y);

                    if (!inverse)
                        J = -Eigen::Vector2d(
                                0.5 * (getPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                       getPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                                0.5 * (getPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                       getPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
                                );
                    else if (iter == 0) // 雅各比用I1(x, y)代替，保持不变
                        J = -Eigen::Vector2d(
                                0.5 * (getPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                       getPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                0.5 * (getPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                       getPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1))
                        );

                    if (!inverse || iter == 0)
                        H += J * J.transpose();
                    b += - error * J;
                    cost += error * error;
                }

            Eigen::Vector2d update = H.ldlt().solve(b);

            if (std::isnan(update[0]))
            {
                succeed = false;
                std::cout << "update is NAN." << std::endl;
                break;
            }

            if (iter > 0 && cost > last_cost)
                break;

            dx += update[0];
            dy += update[1];
            last_cost = cost;
            succeed = true;

            if (update.norm() < 1e-2)
                break;
        }

        success[i] = succeed;

        kp2[i].pt = kp1[i].pt + cv::Point2f(dx, dy);
    }
}


void opticalFlowSingleLevel(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const std::vector<cv::KeyPoint> & kp1,
        std::vector<cv::KeyPoint> & kp2,
        std::vector<bool> & success,
        bool inverse,
        bool has_initial)
{
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    cv::parallel_for_(cv::Range(0, kp1.size()),
                      [ObjectPtr = &tracker](auto && PH1) { ObjectPtr->calculateOpticalFlow(std::forward<decltype(PH1)>(PH1)); });
}


void opticalFlowMultiLevel(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const std::vector<cv::KeyPoint> & kp1,
        std::vector<cv::KeyPoint> & kp2,
        std::vector<bool> & success,
        bool inverse)
{
    // params
    int pyramids = 4;
    double pyramid_scale = 0.5;

    // build pyramids
    std::vector<cv::Mat> pyr1, pyr2;
    for (int i = 0; i < pyramids; ++i)
    {
        if (i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            cv::Mat img1_pry, img2_pry;
            cv::resize(pyr1[i - 1], img1_pry, cv::Size(pyr1[i-1].cols * pyramid_scale, pyr1[i-1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pry, cv::Size(pyr2[i-1].cols * pyramid_scale, pyr2[i-1].rows * pyramid_scale));
            pyr1.push_back(img1_pry);
            pyr2.push_back(img2_pry);
        }
    }

    // course-to-fine LK tracking
    std::vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    for (auto kp : kp1)
    {
        auto kp_top = kp;
        kp_top.pt *= pow(0.5, pyramid_scale - 1);
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--)
    {
        success.clear();
        opticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);

        if (level > 0)
        {
            for (auto & kp : kp1_pyr)
                kp.pt /= pyramid_scale;
            for (auto & kp : kp2_pyr)
                kp.pt /= pyramid_scale;
        }
    }

    for (auto & kp : kp2_pyr)
        kp2.push_back(kp);
}