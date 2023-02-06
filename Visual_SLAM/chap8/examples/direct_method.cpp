/*
 * @Description: 直接法
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-04 21:04:10
 */


#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>

// camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// baseline
double baseline = 0.573;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


class JacobianAccumulator
{
public:
    JacobianAccumulator(
            const cv::Mat & img1,
            const cv::Mat & img2,
            const VecVector2d & pixel_ref,
            const std::vector<double> & depth_ref,
            Sophus::SE3d & T) :
            img1(img1), img2(img2), pixel_ref(pixel_ref), depth_ref(depth_ref), T(T)
    {
        projection = VecVector2d(pixel_ref.size(), Eigen::Vector2d(0, 0));
    }

    void accumulate_jacobian(const cv::Range & range);

    Matrix6d getHessian() const { return H; };
    Vector6d getBias() const { return b; }
    double getCost() const { return cost; }
    VecVector2d getProjectedPoints() const { return projection; }

    void reset()
    {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const cv::Mat & img1;
    const cv::Mat & img2;
    const VecVector2d & pixel_ref;
    const std::vector<double> & depth_ref;
    Sophus::SE3d & T;
    VecVector2d projection; // projected points

    std::mutex hessian_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};


void DirectPoseEstimationSingleLayer(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const VecVector2d & pixel_ref,
        const std::vector<double> & depth_ref,
        Sophus::SE3d & T
);


void DirectPoseEstimationMultiLayer(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const VecVector2d & pixel_ref,
        const std::vector<double> & depth_ref,
        Sophus::SE3d & T
);


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
    cv::Mat left_img = cv::imread("../left.png", cv::IMREAD_UNCHANGED);
    cv::Mat disparity_img = cv::imread("../disparity.png", cv::IMREAD_UNCHANGED);

    cv::RNG rng;
    VecVector2d pixel_ref;
    std::vector<double> depth_ref;

    int points_num = 2000;
    for (int i = 0; i < points_num; ++i)
    {
        int border = 20;
        int x = rng.uniform(border, left_img.cols - border);
        int y = rng.uniform(border, left_img.rows - border);
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity;
        depth_ref.push_back(depth);
        pixel_ref.push_back(Eigen::Vector2d(x, y));
    }

    Sophus::SE3d T;
    for (int i = 1; i < 6; ++i)
    {
        boost::format img_fmt("../%06d.png");
        cv::Mat right_img = cv::imread((img_fmt % i).str(), cv::IMREAD_UNCHANGED);
//        DirectPoseEstimationSingleLayer(left_img, right_img, pixel_ref, depth_ref, T);
        DirectPoseEstimationMultiLayer(left_img, right_img, pixel_ref, depth_ref, T);
    }

    return 0;
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range & range) {

    // parameters
    int half_patch_size = 1;
    int cnt_success;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_temp;

    for (int i = range.start; i < range.end; ++i)
    {
        // 相机坐标系下的点
        Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d((pixel_ref[i][0] - cx) / fx, (pixel_ref[i][1] - cy) / fy, 1);
        Eigen::Vector3d point_cur = T * point_ref;

        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2];
        if (Z < 0)
            continue;

        double u = fx * X / Z + cx, v = fy * Y / Z + cy;
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size || v > img2.rows - half_patch_size)
            continue;

        projection[i] = Eigen::Vector2d(u, v);
        cnt_success++;

        // compute error and jacobian
        for (int x = -half_patch_size; x <= half_patch_size; ++x)
            for (int y = -half_patch_size; y <= half_patch_size; ++y)
            {
                double error = getPixelValue(img1, pixel_ref[i][0] + x, pixel_ref[i][1] + y) -
                               getPixelValue(img2, u + x, v + y);

                Eigen::Vector2d J_img2pixel;
                Matrix26d J_pixel2xi;

                J_img2pixel = Eigen::Vector2d(
                        0.5 * (getPixelValue(img2, u + x + 1, v + y) - getPixelValue(img2, u + x - 1 , v + y)),
                        0.5 * (getPixelValue(img2, u + x, v + y + 1) - getPixelValue(img2, u + x, v + y - 1))
                );

                J_pixel2xi <<
                           fx / Z, 0, -fx * X / (Z * Z), -fx * X * Y / (Z * Z), fx + fx * X * X / (Z * Z), -fx * Y / Z,
                        0, fy / Z, -fy * Y / (Z * Z), -fy - fy * Y * Y / (Z * Z), fy * X * Y / (Z * Z), fy * X / Z;


                Vector6d J = - (J_img2pixel.transpose() * J_pixel2xi).transpose();   // 链式求导

                hessian += J * J.transpose();
                bias += -error * J;
                cost_temp += error * error;
            }
    }

    if (cnt_success)
    {
        // set hessian, bias and cost
        std::unique_lock<std::mutex> lock(hessian_mutex);
        H += hessian;
        b += bias;
        cost += cost_temp / cnt_success;
    }
}


void DirectPoseEstimationSingleLayer(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const VecVector2d & pixel_ref,
        const std::vector<double> & depth_ref,
        Sophus::SE3d & T)
{
    int iterations = 10;
    double cost, last_cost = 0;
    JacobianAccumulator jacob_accu(img1, img2, pixel_ref, depth_ref, T);

    for (int iter = 0; iter < iterations; ++iter)
    {
        jacob_accu.reset();
        cv::parallel_for_(cv::Range(0, pixel_ref.size()),
                          [ObjectPtr = &jacob_accu](auto && PH1) { ObjectPtr->accumulate_jacobian(std::forward<decltype(PH1)>(PH1)); });
        Matrix6d H = jacob_accu.getHessian();
        Vector6d b = jacob_accu.getBias();

        Vector6d update = H.ldlt().solve(b);
        T = Sophus::SE3d::exp(update) * T;
        cost = jacob_accu.getCost();

        if (std::isnan(update[0]))
        {
            std::cout << "update is nan." << std::endl;
            break;
        }

        if (iter > 0 && cost > last_cost)
            break;

        last_cost = cost;
        std::cout << "iteration: " << iter << ", cost: " << cost << std::endl;
    }

    std::cout << "T = \n" << T.matrix() << std::endl;

    // plot the projected pixel
    cv::Mat img2_plot;
    cv::cvtColor(img2, img2_plot, cv::COLOR_GRAY2BGR);
    VecVector2d projection = jacob_accu.getProjectedPoints();

    for (int i = 0; i < pixel_ref.size(); ++i)
    {
        Eigen::Vector2d p_ref = pixel_ref[i], p_cur = projection[i];
        if (p_cur[0] > 0 && p_cur[1] > 0)   // x > 0 && y > 0
        {
//            std::cout << "p_ref = " << p_ref[0] << "\t" << "p_cur = " << p_cur[0] << std::endl;
            cv::circle(img2_plot, cv::Point2d(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 255, 0), 2);
            cv::line(img2_plot, cv::Point2d(p_ref[0], p_ref[1]), cv::Point2d(p_cur[0], p_cur[1]), cv::Scalar(0, 255, 0));
        }
    }
    cv::imshow("current", img2_plot);
    cv::waitKey(0);
}


void DirectPoseEstimationMultiLayer(
        const cv::Mat & img1,
        const cv::Mat & img2,
        const VecVector2d & pixel_ref,
        const std::vector<double> & depth_ref,
        Sophus::SE3d & T)
{
    // params
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

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

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: pixel_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T);
    }
}


