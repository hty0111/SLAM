/*
 * @Description: 根据训练的词典计算相似度评分
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-11-08 12:45:04
 */

#include <DBoW3/DBoW3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>


int main()
{
    DBoW3::Vocabulary vocabulary("../vocabulary.yml.gz");
    assert(!vocabulary.empty() && "vocabulary is empty.");

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

    // compare images directly
    for (int i = 0; i < images.size(); ++i)
    {
        DBoW3::BowVector v1;
        vocabulary.transform(descriptors[i], v1);   // 把ORB特征转换成词向量
        for (int j = i; j < images.size(); ++j)
        {
            DBoW3::BowVector v2;
            vocabulary.transform(descriptors[j], v2);
            double score = vocabulary.score(v1, v2);
            std::cout << "image " << i << " vs image " << j << ": " << score << std::endl;
        }
        std::cout << std::endl;
    }

    // compare images by database
    DBoW3::Database database(vocabulary, false, 0);
    for (auto & descriptor : descriptors)
        database.add(descriptor);
    std::cout << "database info: " << database << std::endl << std::endl;
    for (int i = 0; i < descriptors.size(); ++i)
    {
        DBoW3::QueryResults results;
        database.query(descriptors[i], results, 4);
        std::cout << "image " << i << " returns " << results << std::endl << std::endl;
    }

    return 0;
}
