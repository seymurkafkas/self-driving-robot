#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include "polynomial_fit.h"

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
    cv::Point topLeftPoint(0, 0);
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
 * Find the lowermost lane windows, to be used in the Sliding Window Method
 *
 * @param binaryImage Binary image on which histogram poeration will be applied.
 * @param histogramSize The number of bins to which the X axis is partitioned.
 * @return Two lowermost rectangles containing the lane points
 */
std::pair<cv::Rect, cv::Rect> getLowermostLaneRegions(cv::Mat binaryImage, int histogramSize, int verticalSize)
{

    cv::Size imageSize = binaryImage.size();
    int rectangleHeight = imageSize.height / verticalSize;
    int rectangleWidth = imageSize.width / histogramSize;

    std::vector<int> lanePointDistributionVector = getPointDistribution(binaryImage, histogramSize, verticalSize);

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
        //      std::cout << "Current: " << lanePointDistributionVector[i] << std::endl;
        //     std::cout << "maxIndex: " << maxIndex << std::endl;
        //     std::cout << "Second max index: " << secondMaxIndex << std::endl;

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
        //      std::cout << "At index " << i << ": " << cvRound(histogram.at(i - 1)) << std::endl;
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

        cv::findNonZero(subRegion, detectedLanePoints); //Get all non-black pixels. All are white in our case
        float sumOfXCoordinates = 0.0f;

        for (int i = 0; i < detectedLanePoints.size(); ++i) //Calculate average X position
        {
            float x = detectedLanePoints[i].x;
            sumOfXCoordinates += window.x + x;
        }

        float averageXCoordinate = detectedLanePoints.empty() ? currentX : sumOfXCoordinates / detectedLanePoints.size();

        cv::Point point(averageXCoordinate, window.y + window.height * 0.5f);
        gatheredPoints.push_back(point);

        //Move the window up
        window.y -= window.height;

        //For the uppermost position
        if (window.y < 0)
        {
            window.y = 0;
            reachedUpperBoundary = true;
        }

        //Move x position
        window.x += (point.x - currentX);

        //Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix
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
    // todo do some checks on input.

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

    std::pair<cv::Rect, cv::Rect> rectRegions = getLowermostLaneRegions(projectedImage, 5, 8);

    std::vector<cv::Point2f> firstCurvePointCluster = slidingWindowMethod(projectedImage, rectRegions.first);
    std::vector<cv::Point2f> secondCurvePointCluster = slidingWindowMethod(projectedImage, rectRegions.second);

    // Curve fit for both

    PolynomialRegression<float> leastSquareSum;
    std::vector<float> coefficientsForLeftQuadratic, coefficientsForRightQuadratic;

    leastSquareSum.fitIt(firstCurvePointCluster, 2, coefficientsForLeftQuadratic);
    leastSquareSum.fitIt(secondCurvePointCluster, 2, coefficientsForRightQuadratic);

    /*     std::cout << "a: " << coefficientsForLeftQuadratic[0] << "  b: " << coefficientsForLeftQuadratic[1] << "   c: " << coefficientsForLeftQuadratic[2] << std::endl;
    std::cout << "2nd ONE: " << std::endl;
    std::cout << "a: " << coefficientsForRightQuadratic[0] << "  b: " << coefficientsForRightQuadratic[1] << "   c: " << coefficientsForRightQuadratic[2] << std::endl; */

    float firstXIntercept = calculateXIntercept(projectedImage.size(), coefficientsForLeftQuadratic);
    float secondXIntercept = calculateXIntercept(projectedImage.size(), coefficientsForRightQuadratic);

    float error = ((firstXIntercept + secondXIntercept) / 2) - (projectedImage.size().width / 2);
    //Find x intersection
    std::cout << "Error :" << error << std::endl;
    return error;
    //Return (xIntersectionFirst+XıntersectionSecond/2 )-(rawImage.size().width/2)
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
        cv::Mat maskedImage = maskImage(lineImage);
        cv::Mat projectedImage = projectImage(lineImage);

        std::vector<int> histogram = getPointDistribution(projectedImage, 10, 10);
        cv::Mat visualHistogram = getVisualisedHistogram(histogram, 10);

        std::pair<cv::Rect, cv::Rect> rectRegions = getLowermostLaneRegions(projectedImage, 5, 8);

        // cv::rectangle(projectedImage, rectRegions.first, cv::Scalar(255, 0, 0));
        // cv::rectangle(projectedImage, rectRegions.second, cv::Scalar(255, 0, 0));

        // std::vector< cv::Point2f > gatherPoints= slidingWindowMethod(projectedImage,cv::Rect(cv::Point(55,150),cv::Point(65,160)));
        calculateDistanceToLaneCenter(cameraImage);
        cv::imshow("view", projectedImage);
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
    /*     PolynomialRegression<float> leastSquareSum;
    std::vector<cv::Point2f> testPoints;
    testPoints.push_back(cv::Point2f(250, 10));
    testPoints.push_back(cv::Point2f(5250, 50));
    testPoints.push_back(cv::Point2f(13200, 80));
    std::vector<float> coefficients;
    leastSquareSum.fitIt(testPoints, 2, coefficients);
    std::cout << "a: " << coefficients[0] << "  b: " << coefficients[1] << "   c: " << coefficients[2] << std::endl; */
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, rawImageCallback);
    ros::spin();
    cv::destroyWindow("view");
}