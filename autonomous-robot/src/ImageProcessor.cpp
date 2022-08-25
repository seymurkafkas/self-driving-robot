#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include "Polynomial.h"
#include "geometry_msgs/Twist.h"
#include "PIDController.h"
#include "std_msgs/Int32.h"

geometry_msgs::Twist motor_command;
ros::Publisher motor_command_publisher;
PIDController *angularVelocityController;

int robotState = 2; // 0: Halt, 1: Drive at 50% Speed , 2: Drive at 100% Speed
float baseLinearVelocity = 0.40;

//0 Default (No sign is detected)
//3 Stop Sign
//2 Speed Sign
//1 Slow Sign

//Start->  1

//Triangle -> 2

//Hexagon -> 3

enum Sign
{
    noSignDetected,
    slowSign,
    speedSign,
    stopSign
};

/**
 * Get the next state, given the detected sign
 *
 * @param detectedSign the integer identifier of the detected sign.
 * @return The integer identifier of the next state.
 */

int stateTransitionFunction(Sign detectedSign)
{

    if (detectedSign == noSignDetected)
    {
        return robotState;
    }

    if (detectedSign == stopSign)
    {
        std::cout << "Stopping..." << std::endl;
        return 0;
    }
    if (detectedSign == speedSign)
    {
        std::cout << "100% Speed" << std::endl;
        return 2;
    }
    if (detectedSign == slowSign)
    {
        std::cout << "50% Speed" << std::endl;
        return 1;
    }
}

void changeState(Sign detectedSign)
{

    robotState = stateTransitionFunction(detectedSign);
}

float robotStateMultiplier()
{
    float multiplier = 0;
    switch (robotState)
    {
    case 0:
        multiplier = 0;
        break;
    case 1:
        multiplier = 0.5;
        break;
    case 2:
        multiplier = 1;
        break;
    }
    return multiplier;
}

/**
 * Find the number of nonblack pixels in each X Partition and return them in an array
 *
 * @param binaryImage Binary image which will be partitioned
 * @param histogramSize The number of bins to which the X axis is partitioned.
 * @return A vector of counted points at every given bin
 */

std::vector<int> getPointDistribution(cv::Mat binaryImage, int histogramSize, int verticalSize)
{

    cv::Size imageSize = binaryImage.size();
    int rectangleHeight = imageSize.height / verticalSize;
    int rectangleWidth = imageSize.width / histogramSize;
    std::vector<int> pointDistributionAcrossX;
    cv::Point topLeftPoint(0, imageSize.height - rectangleHeight);
    cv::Mat histogram;
    cv::Point bottomRightPoint(rectangleWidth, imageSize.height);
    cv::Rect verticalBin(topLeftPoint, bottomRightPoint);
    //   std::cout << "------------------------------" << std::endl;
    for (int i = 0; i < histogramSize; i++)
    {
        int currentPointCount = cv::countNonZero(binaryImage(verticalBin));
        pointDistributionAcrossX.push_back(currentPointCount);
        //      std::cout << currentPointCount << std::endl;
        verticalBin.x += rectangleWidth;
    }

    return pointDistributionAcrossX;
}

/**
 * Find the bin indices for the most populated two bins in histogram.
 *
 * @param lanePointDistributionVector The vector holding the counted lane pixels per bin.
 * @return The pair of max and second max indices (maxIndex,secondMaxIndex)
 */

std::pair<int, int> getTwoLocalPeaksOnHistogram(std::vector<int> lanePointDistributionVector)
{
    int maxNumberOfPoints, secondMaxNumberOfPoints, maxIndex, secondMaxIndex;

    if (std::max(lanePointDistributionVector[0], lanePointDistributionVector[1]) == lanePointDistributionVector[0])
    {

        maxIndex = 0;
        secondMaxIndex = 1;
        maxNumberOfPoints = lanePointDistributionVector[0];
        secondMaxNumberOfPoints = lanePointDistributionVector[1];
    }
    else
    {

        maxIndex = 1;
        secondMaxIndex = 0;
        maxNumberOfPoints = lanePointDistributionVector[1];
        secondMaxNumberOfPoints = lanePointDistributionVector[0];
    }

    for (int i = 2; i < lanePointDistributionVector.size(); i++)
    {

        if (maxNumberOfPoints <= lanePointDistributionVector[i])
        {
            secondMaxNumberOfPoints = maxNumberOfPoints;
            maxNumberOfPoints = lanePointDistributionVector[i];
            secondMaxIndex = maxIndex;
            maxIndex = i;
        }
        else if (secondMaxNumberOfPoints < lanePointDistributionVector[i])
        {
            secondMaxNumberOfPoints = lanePointDistributionVector[i];
            secondMaxIndex = i;
        }
    }

    return std::make_pair(maxIndex, secondMaxIndex);
}

/**
 * Find the lowermost lane windows, to be used in the Sliding Window Method
 *
 * @param binaryImage Binary image on which histogram operation will be applied.
 * @param histogramSize The number of bins to which the X axis is partitioned.
 * @return Two lowermost rectangles containing the lane points
 */

std::pair<cv::Rect, cv::Rect> getLowermostLaneRegions(cv::Mat binaryImage, int histogramSize, int verticalSize)
{

    cv::Size imageSize = binaryImage.size();
    int rectangleHeight = imageSize.height / verticalSize;
    int rectangleWidth = imageSize.width / histogramSize;

    std::vector<int> lanePointDistributionVector = getPointDistribution(binaryImage, histogramSize, verticalSize);

    std::pair<int, int> indicesForLocalPeaks = getTwoLocalPeaksOnHistogram(lanePointDistributionVector);
    int maxIndex = indicesForLocalPeaks.first;
    int secondMaxIndex = indicesForLocalPeaks.second;

    int yTopLeftPoint = imageSize.height - rectangleHeight;
    int xTopLeftPoint = maxIndex * (rectangleWidth);
    int yBottomRightPoint = imageSize.height;
    int xBottomRightPoint = (maxIndex + 1) * (rectangleWidth);
    cv::Rect firstRectangle(cv::Point(xTopLeftPoint, yTopLeftPoint), cv::Point(xBottomRightPoint, yBottomRightPoint));

    yTopLeftPoint = imageSize.height - rectangleHeight;
    xTopLeftPoint = secondMaxIndex * (rectangleWidth);
    yBottomRightPoint = imageSize.height;
    xBottomRightPoint = (secondMaxIndex + 1) * (rectangleWidth);
    cv::Rect secondRectangle(cv::Point(xTopLeftPoint, yTopLeftPoint), cv::Point(xBottomRightPoint, yBottomRightPoint));

    return std::make_pair(firstRectangle, secondRectangle);
}

cv::Mat getVisualisedHistogram(std::vector<int> histogram, int histogramSize)
{
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound((double)hist_w / histogramSize);
    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    float maximumPointCount = *std::max_element(histogram.begin(), histogram.end());
    for (int i = 1; i < histogramSize; i++)
    {
        cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(histogram.at(i - 1)) * hist_h / maximumPointCount),
                 cv::Point(bin_w * (i), hist_h - cvRound(histogram.at(i)) * hist_h / maximumPointCount),
                 cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    return histImage;
}

/**
 * Get the relevant lane points from the image using the Sliding Window method.
 *
 * @param image Image the points of which will be filtered and transformed.
 * @param window The rectangle indicating the starting window.
 * @return A vector collection of points which were extracted starting from the initial window
 */

std::vector<cv::Point2f> slidingWindowMethod(cv::Mat image, cv::Rect window)
{
    std::vector<cv::Point2f> gatheredPoints;
    const cv::Size imageSize = image.size();
    bool reachedUpperBoundary = false;

    while (true)
    {
        float currentX = window.x + window.width * 0.5f;

        cv::Mat subRegion = image(window); //Extract region of interest
        std::vector<cv::Point> detectedLanePoints;

        cv::findNonZero(subRegion, detectedLanePoints); //Get all lane pixels
        float sumOfXCoordinates = 0.0f;

        for (int i = 0; i < detectedLanePoints.size(); ++i) //Calculate X mean of points
        {
            float x = detectedLanePoints[i].x;
            sumOfXCoordinates += window.x + x;
        }

        float averageXCoordinate = detectedLanePoints.empty() ? currentX : sumOfXCoordinates / detectedLanePoints.size();

        cv::Point point(averageXCoordinate, window.y + window.height * 0.5f);

        if (!detectedLanePoints.empty())
        {
            gatheredPoints.push_back(point);
        }

        // Shift the window up
        window.y -= window.height;

        //Check if the window reached the upper end
        if (window.y < 0)
        {
            window.y = 0;
            reachedUpperBoundary = true;
        }

        //Shift X
        window.x += (point.x - currentX);

        //Make sure the window doesn't overflow, otherwise segmentation fault occurs
        if (window.x < 0)
            window.x = 0;
        if (window.x + window.width >= imageSize.width)
            window.x = imageSize.width - window.width - 1;

        if (reachedUpperBoundary)
            break;
    }

    return gatheredPoints;
}

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
    cv::Point2f source_points[4];
    cv::Point2f dest_points[4];

    source_points[0] = cv::Point2f(0, 600);
    source_points[1] = cv::Point2f(232, 425);
    source_points[2] = cv::Point2f(557, 425);
    source_points[3] = cv::Point2f(800, 600);

    dest_points[0] = cv::Point2f(0, 600);
    dest_points[1] = cv::Point2f(0, 0);
    dest_points[2] = cv::Point2f(800, 0);
    dest_points[3] = cv::Point2f(800, 600);

    cv::Mat dst;
    cv::Mat transformMatrix = cv::getPerspectiveTransform(source_points, dest_points);
    cv::warpPerspective(image, dst, transformMatrix, image.size());
    return dst;
}

float calculateXIntercept(cv::Size imageSize, const std::vector<float> &polynomialCoefficients)
{

    float c = polynomialCoefficients[0];
    float b = polynomialCoefficients[1];
    float a = polynomialCoefficients[2];
    float y = imageSize.height;
    float xAtIntercept = (a * y * y + b * y + c);
    return xAtIntercept;
}

float calculateXatGivenY(float y, const std::vector<float> &polynomialCoefficients)
{

    float c = polynomialCoefficients[0];
    float b = polynomialCoefficients[1];
    float a = polynomialCoefficients[2];
    float xResult = (a * y * y + b * y + c);
    return xResult;
}

void addPolynomialCurveToImage(cv::Mat editedImage, std::vector<float> &coefficients)
{
    if (coefficients.empty())
    {
        return;
    }
    std::vector<cv::Point> polynomialCurve;

    for (int y = editedImage.size().height; y >= 0; y -= 10)
    {

        cv::Point pointOnLine(calculateXatGivenY(y, coefficients), y);
        polynomialCurve.push_back(pointOnLine);
    }

    cv::polylines(editedImage, polynomialCurve, false, cv::Scalar(255, 0, 255), 3);
}

/**
 * Get the distance to the center of the two lanes (Error Signal)
 *
 * @param rawImage The raw image with visible lane markers.
 * @return The distance (error) in length units
 */

float calculateDistanceToLaneCenter(cv::Mat rawImage)
{

    cv::Mat grayImageMatrix;
    cv::cvtColor(rawImage, grayImageMatrix, cv::COLOR_BGR2GRAY);
    cv::Mat blurredImage;
    cv::GaussianBlur(grayImageMatrix, blurredImage, cv::Size(5, 5), 1);
    cv::Mat lineImage;
    cv::Canny(blurredImage, lineImage, 100, 150, 5, true);
    cv::Mat maskedImage = maskImage(lineImage);
    cv::Mat projectedImage = projectImage(lineImage);

    //Count of lane points after diving the image into vertical bins
    //std::vector<int> histogram = getPointDistribution(projectedImage, 10, 10);

    //
    //cv::Mat visualHistogram = getVisualisedHistogram(histogram, 10);

    std::pair<cv::Rect, cv::Rect> rectRegions = getLowermostLaneRegions(projectedImage, 4, 12);

    std::vector<cv::Point2f> firstCurvePointCluster = slidingWindowMethod(projectedImage, rectRegions.first);
    std::vector<cv::Point2f> secondCurvePointCluster = slidingWindowMethod(projectedImage, rectRegions.second);

    // Curve fit for both

    PolynomialRegression<float> leastSquareSum;
    std::vector<float> coefficientsForLeftQuadratic, coefficientsForRightQuadratic;

    bool firstLaneWithinBounds = (!firstCurvePointCluster.empty());

    bool secondLaneWithinBounds = (!secondCurvePointCluster.empty());
    if (firstLaneWithinBounds)
    {
        leastSquareSum.fitIt(firstCurvePointCluster, 2, coefficientsForLeftQuadratic);
    }

    if (secondLaneWithinBounds)
    {
        leastSquareSum.fitIt(secondCurvePointCluster, 2, coefficientsForRightQuadratic);
    }

    // Plot the curve in color in the projected Image
    cv::Mat colouredProjectedImage;
    cv::cvtColor(projectedImage, colouredProjectedImage, cv::COLOR_GRAY2BGR);
    addPolynomialCurveToImage(colouredProjectedImage, coefficientsForLeftQuadratic);
    addPolynomialCurveToImage(colouredProjectedImage, coefficientsForRightQuadratic);
    cv::rectangle(colouredProjectedImage, rectRegions.first, cv::Scalar(224, 255, 255));
    cv::rectangle(colouredProjectedImage, rectRegions.second, cv::Scalar(224, 255, 255));
    //

    cv::rectangle(projectedImage, rectRegions.first, cv::Scalar(255, 0, 0));
    cv::rectangle(projectedImage, rectRegions.second, cv::Scalar(255, 0, 0));

    cv::imshow("projection", colouredProjectedImage);
    float firstXIntercept, secondXIntercept;
    if (firstLaneWithinBounds)
    {
        firstXIntercept = calculateXIntercept(projectedImage.size(), coefficientsForLeftQuadratic);
    }
    if (secondLaneWithinBounds)
    {
        secondXIntercept = calculateXIntercept(projectedImage.size(), coefficientsForRightQuadratic);
    }
    else
    {
        secondXIntercept = (firstXIntercept >= (rawImage.size().width / 2)) ? 0 : rawImage.size().width;
    }

    float error = (projectedImage.size().width / 2) - ((firstXIntercept + secondXIntercept) / 2);


 


    return error;
}

void rawImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat cameraImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        float errorSignal = calculateDistanceToLaneCenter(cameraImage);

        if (robotState == 0)
        {
            motor_command.linear.x = baseLinearVelocity * robotStateMultiplier();
            motor_command.angular.z = 0;
        }
        else
        {

            motor_command.linear.x = baseLinearVelocity * robotStateMultiplier();
            motor_command.angular.z = angularVelocityController->getPIDOutput(errorSignal);
        }

        motor_command_publisher.publish(motor_command);

        cv::waitKey(30);
    }

    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void stateCallback(const std_msgs::Int32::ConstPtr &sign)
{
    int detectedSign(sign->data);
    changeState(Sign(detectedSign));
}

int main(int argc, char **argv)
{
    angularVelocityController = new PIDController();
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    cv::namedWindow("Turtlebot");
    motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, rawImageCallback);
    ros::Subscriber stateSub = nh.subscribe("/state", 1, stateCallback);
    ros::spin();
    cv::destroyWindow("view");
}