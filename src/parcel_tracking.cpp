#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv.hpp>




using namespace std;

#define markerPath "/home/martin/Parcel Tracking Bags/Markers/"

vector<cv::KeyPoint> markerKeypoints;
//cv::FlannBasedMatcher matcher;
cv::Mat markerImg;
cv::Mat markerDescriptor;

cv::Ptr<cv::ORB> orb;
//cv::OrbDescriptorExtractor extractor;

void imageCallback(const sensor_msgs::ImagePtr img)
{
    ROS_INFO("Image received");
    cv::Mat imgMat;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
    //cv::namedWindow("input",cv::WINDOW_NORMAL);
    //cv::imshow("input",cv_ptr->image);
    //cv::waitKey();
    cv::Mat greyScene;
    cv::cvtColor(cv_ptr->image, greyScene, CV_BGR2GRAY);


    // find feature
    vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat sceneDescriptor;

    orb->detect(greyScene , sceneKeypoints);
    orb->compute( greyScene, sceneKeypoints, sceneDescriptor );
    cout << "keypoints marker and scene: " << markerKeypoints.size() << "  " << sceneKeypoints.size() << endl;
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( markerDescriptor, sceneDescriptor, matches );
    cv::Mat withKeypoints;//(1000,1200,CV_8UC3,cv::Scalar(255,255,255));
    cout << "Matches: " << matches.size() << endl;
    cv::imshow("marker",markerImg);
    //cv::waitKey();
    cv::imshow("scene",cv_ptr->image);
    //cv::waitKey();
    vector<char> temp;
    cv::drawMatches(markerImg,markerKeypoints,cv_ptr->image,sceneKeypoints,matches,withKeypoints,cv::Scalar(255,0,0),cv::Scalar(0,255,0),temp, cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("matches",withKeypoints);
    cv::waitKey();

}

void pointCloudCallback(const sensor_msgs::PointCloud2Ptr pointCloud)
{
    ROS_INFO("Pointcloud received");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "parcel_tracker");
    ros::NodeHandle n;

    // Parameters
    cout << " new node" << endl;

    // Subscribe
    ros::Subscriber imageSubscriber = n.subscribe("/camera/rgb/image_color",1,imageCallback);
    ros::Subscriber pointCloudSubscriber = n.subscribe("/camera/depth/points",1,pointCloudCallback);

    string loadImg = std::string(markerPath)+std::string("markerTest.jpg");
    cout << loadImg << endl;

    // Obtain markers and extract keypoints
    markerImg = cv::imread(loadImg,CV_LOAD_IMAGE_GRAYSCALE);

   orb = cv::ORB::create();
   orb->detect(markerImg,markerKeypoints);
   orb->compute(markerImg,markerKeypoints,markerDescriptor);

    //cv::OrbFeatureDetector *detector = new cv::OrbFeatureDetector(200);
    //cv::SurfFeatureDetector detector;
    //detector->compute(markerImg,markerKeypoints,markerDescriptor);
    //extractor.compute( markerImg, markerKeypoints, markerDescriptor );

    cout << "marker keypoints: " << markerKeypoints.size() << endl;
    //cout << "markerImg: " << markerImg.rows << "  " << markerImg.cols << endl;

   // cv::imshow("markerGrey",markerImg);
   // cv::waitKey();

    //Spin
    ros::Rate r(10);
    while (ros::ok()) {

        ros::spinOnce();
        r.sleep();
        // cv::imshow("Lines",currentImg);
    }
}
