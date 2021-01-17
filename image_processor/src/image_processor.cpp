#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

cv::Mat maskImage(cv::Mat image)
{
    cv::Mat maskedImage;
    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());
    cv::Point polygonBoundaries[4] = {
        cv::Point(30, 410),
        cv::Point(150, 245),
        cv::Point(480, 245),
        cv::Point(610, 410)};

    // Polygon mask
    cv::fillConvexPoly(mask, polygonBoundaries, 4, cv::Scalar(255, 255, 255));
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(image, mask, maskedImage);

    return maskedImage;
}

cv::Mat projectImage(const cv::Mat &image)

{
    // todo do some checks on input.

    cv::Point2f source_points[4];
    cv::Point2f dest_points[4];

    source_points[0] = cv::Point2f(50, 420);
    source_points[1] = cv::Point2f(135, 350);
    source_points[2] = cv::Point2f(490, 350);
    source_points[3] = cv::Point2f(575, 420);

    dest_points[0] = cv::Point2f(0, 480);
    dest_points[1] = cv::Point2f(0, 245);
    dest_points[2] = cv::Point2f(640, 245);
    dest_points[3] = cv::Point2f(640, 480);

    cv::Mat dst;
    cv::Mat transformMatrix = cv::getPerspectiveTransform(source_points, dest_points);
    cv::warpPerspective(image, dst, transformMatrix, image.size());

    return dst;
}

void rawImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat cameraImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat grayImageMatrix;

        cv::cvtColor(cameraImage, grayImageMatrix, cv::COLOR_BGR2GRAY);
        cv::Mat blurredImage;
        cv::GaussianBlur(grayImageMatrix, blurredImage, cv::Size(5, 5), 1);
        cv::Mat lineImage;
        cv::Canny(blurredImage, lineImage, 100, 150, 5, true);
        cv::Mat maskedImage = maskImage(cameraImage);

        cv::Mat projectedImage = projectImage(cameraImage);
        cv::imshow("view", cameraImage);
        cv::waitKey(30);
    }

    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    cv::namedWindow("Turtlebot");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, rawImageCallback);
    ros::spin();
    cv::destroyWindow("view");
}