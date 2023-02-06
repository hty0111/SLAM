/*
 * @Description: 训练词典
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-08 11:20:30
 */

#include <DBoW3/DBoW3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>


int main()
{
    std::vector<cv::Mat> images;
    for (int i = 0; i < 10; ++i)
    {
        std::string path = "../data/" + std::to_string(i + 1) + ".png";
        images.push_back(cv::imread(path));
    }

    // detect ORB features
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    std::vector<cv::Mat> descriptors;
    for (const cv::Mat & image : images)
    {
        std::vector<cv::KeyPoint> key_points;
        cv::Mat descriptor;
        detector->detectAndCompute(image, cv::Mat(), key_points, descriptor);
        descriptors.push_back(descriptor);
    }

    // create vocabulary
    DBoW3::Vocabulary vocabulary;
    vocabulary.create(descriptors);
    std::cout << "vocabulary info: " << vocabulary << std::endl;
    vocabulary.save("../vocabulary.yml.gz");

    return 0;
}