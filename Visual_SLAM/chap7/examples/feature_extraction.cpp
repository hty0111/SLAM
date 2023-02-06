/*
 * @Description: orb匹配
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-04-25 19:13:34
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>

int main()
{
    // 读取图像
    cv::Mat img1 = cv::imread("../1.png");
    cv::Mat img2 = cv::imread("../2.png");

    // 初始化
    std::vector<cv::KeyPoint> key_points1, key_points2;
    cv::Mat descriptions1, descriptions2;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2,
                                           cv::ORB::HARRIS_SCORE,31,20);

    // (1) 检测 Oriented FAST 角点位置
    orb->detect(img1, key_points1);
    orb->detect(img2, key_points2);

    // (2) 由角点位置计算 BRIEF 描述子
    orb->compute(img1, key_points1, descriptions1);
    orb->compute(img2, key_points2, descriptions2);

    cv::Mat output_img1;
    drawKeypoints(img1, key_points1, output_img1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    imshow("ORB特征点", output_img1);

    // (3) 用 Hamming 距离匹配两帧中的 BRIEF
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptions1, descriptions2, matches);

    // (4) 筛选配对点
    // 找出最大和最小距离
    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    std::cout << "Max dist: " << max_dist << std::endl << "Min dist: " << min_dist << std::endl;

    // 描述子距离大于两倍最小值或经验值时，认为匹配有误
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptions1.rows; i++)
        if (matches[i].distance <= std::max(2 * min_dist, 30.0))
            good_matches.push_back(matches[i]);

    // (5) 绘制匹配结果
    cv::Mat img_match, img_good_match;
    drawMatches(img1, key_points1, img2, key_points2, matches, img_match);
    drawMatches(img1, key_points1, img2, key_points2, good_matches, img_good_match);
    imshow("所有匹配点", img_match);
    imshow("优化后的匹配点", img_good_match);
    cv::waitKey(0);

    return 0;
}
