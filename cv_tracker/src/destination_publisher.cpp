#include "opencv2/core.hpp"
// Include opencv libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// include standard library in c++
#include <iostream>
#include <sstream>
#include <fstream>

// include ros library
#include "ros/ros.h"

// include destination message
#include <cv_tracker/destination_msg.h>
#include <ros/package.h>

using namespace std;
using namespace cv;
using namespace Eigen;

// calibration parameters
// square width and height are both 20 milimeters
const float calibrationSquareDimension = 0.020f; //meters

// aruco markers' width and height are both 160 milimeters
const float arucoSquareDimension = 0.16f;

// number of corners of the calibration chess board
const Size chessboardDimensions = Size(7,9);
std::string packagePath = ros::package::getPath("cv_tracker");

/** Create corner positions on the chessboard and store them in variable "corners"
 *  Size:
 *
 */
void createKnowBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
    for(int i = 0; i < boardSize.height; i++)
    {
        for(int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));

        }
    }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false)
{
    for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, Size(7,9), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            allFoundCorners.push_back(pointBuf);
        }

        if (showResults)
        {
            drawChessboardCorners(*iter, Size(7,9), pointBuf, found);
            cv::imshow("Looking for Corners", *iter);
            waitKey(0);
        }

    }
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{
    vector<vector<Point2f>> checkerboardImagesSpacePoints;
    getChessboardCorners(calibrationImages, checkerboardImagesSpacePoints, false);

    vector<vector<Point3f>> worldSpaceCornerPoints(1);

    createKnowBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImagesSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<Mat> rVectors, tVectors;
    distanceCoefficients = Mat::zeros(8, 1, CV_64F);
    calibrateCamera(worldSpaceCornerPoints, checkerboardImagesSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);

}

void Vector2Transformation(Vec3d& rotVec, Vec3d& transVec, Matrix4d& output)
{
    Matrix4d rotationMatrix;
    Mat rotMat_cv = Mat::zeros(3,3,CV_64F);
    Rodrigues(rotVec, rotMat_cv);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            output(i,j) = rotMat_cv.at<double>(i,j);
        }
    }

    output(0,3) = transVec[0];
    output(1,3) = transVec[1];
    output(2,3) = transVec[2];
    output(3,0) = 0.0;
    output(3,1) = 0.0;
    output(3,2) = 0.0;
    output(3,3) = 1.0;

}

vector<Matrix4d> startWebCamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimensions,vector<int> &MarkerIds)
{
    Mat frame;

    vector<int> markerIds;
    vector<Matrix4d> tf_marker2cam;
    Matrix4d tmpMat;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    VideoCapture vid(0);

    if(!vid.isOpened())
    {
        tmpMat << 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
        tf_marker2cam.push_back(tmpMat);
        return tf_marker2cam;
    }

    namedWindow("Webcam", WINDOW_NORMAL | WINDOW_KEEPRATIO);

    vector<Vec3d> rotationVectors, translationVectors;
    // FOR DEBUG
    //namedWindow("cameraMatrix", WINDOW_NORMAL | WINDOW_KEEPRATIO);
    //imshow("cameraMatrix", cameraMatrix);
    //namedWindow("distanceCoefficients", WINDOW_NORMAL | WINDOW_KEEPRATIO);
    //imshow("distanceCoefficients", distanceCoefficients);



    if (!vid.read(frame))
    {
        tmpMat << 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
        tf_marker2cam.push_back(tmpMat);
        return tf_marker2cam;
    }

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    MarkerIds = markerIds;
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimensions,
                                     cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

    for(int i = 0; i < markerIds.size(); i++)
    {
//        cout << "Rotation vector NO. " << i << "\n";
//        cout << rotationVectors[i] << "\n" << endl;
//        cout << "Translation vector NO. " << i << "\n";
//        cout << translationVectors[i] << "\n";
        aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        Vector2Transformation(rotationVectors[i], translationVectors[i], tmpMat);
        tf_marker2cam.push_back(tmpMat);


    }
    imshow("Webcam", frame);
    if(waitKey(1000) >= 0)
    {
        tmpMat << 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
        tf_marker2cam.push_back(tmpMat);
        return tf_marker2cam;
    }


    return tf_marker2cam;

}



bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
    ofstream outStream(name);
    if (outStream)
    {
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        outStream << rows << endl;
        outStream << columns << endl;

        for (int r = 0; r < rows; r++)
        {
            for(int c = 0; c < columns; c++)
            {
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        outStream << rows << endl;
        outStream << columns << endl;

        for (int r = 0; r < rows; r++)
        {
            for(int c = 0; c < columns; c++)
            {
                double value = distanceCoefficients.at<double>(r, c);
                outStream << value << endl;
            }
        }

        outStream.close();
        return true;


    }
    return false;

}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients)
{
    ifstream inStream(name);
    if(inStream)
    {
        uint16_t rows;
        uint16_t columns;

        inStream >> rows;
        inStream >> columns;

        cameraMatrix = Mat(Size(columns, rows), CV_64F);

        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c <columns; c++)
            {
                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r, c) = read;
                cout << cameraMatrix.at<double>(r, c) << "\n";

            }
        }

        // Distance Coefficients
        inStream >> rows;
        inStream >> columns;

        distanceCoefficients = Mat::zeros(rows, columns, CV_64F);
        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c <columns; c++)
            {
                double read = 0.0f;
                inStream >> read;
                distanceCoefficients.at<double>(r, c) = read;
                cout << distanceCoefficients.at<double>(r, c) << "\n";

            }
        }
        inStream.close();
        return true;
    }
    return false;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients)
{
    Mat frame;
    Mat drawToFrame;

    vector<Mat> savedImages;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if (!vid.isOpened())
    {
        return;
    }

    int framesPerSecond = 20;

    namedWindow("Webcam", WINDOW_AUTOSIZE);

    while (true)
    {
        if (!vid.read(frame))
            break;

        vector<Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if (found)
        {
            imshow("Webcam", drawToFrame);
        }
        else
            imshow("Webcam", frame);
        char character = waitKey(1000 / framesPerSecond);

        switch(character)
        {
            case ' ':
                //saving image
                if (found)
                {
                    Mat temp;
                    static int i = 0;
                    string img("img_");
                    frame.copyTo(temp);
                    savedImages.push_back(temp);
                    imwrite(img.append(to_string(i++)).append(".jpg"),temp);
                }
                break;
            case 13:
                //start calibration
                if(savedImages.size() > 15)
                {
                    cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
                    saveCameraCalibration(packagePath + "/IloveCameraCalibration", cameraMatrix, distanceCoefficients);

                }

                break;
            case 27:
                //exit
                return;
                break;
            case 'a':
                cout << savedImages.size() << "\n" << endl;
                break;
        }

    }

}

bool loadTransformationMatrix(Matrix4d& TransformationMatrix, string name)
{
    ifstream inStream(name);
    if(inStream)
    {
        uint16_t rows;
        uint16_t columns;

        inStream >> rows;
        inStream >> columns;

        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c <columns; c++)
            {
                double read = 0.0f;
                inStream >> read;
                TransformationMatrix(r,c) = read;
                cout << TransformationMatrix(r, c) << ",";
            }
            cout << "\n";
        }

        inStream.close();
        return true;
    }
    return false;

}



int main(int argc, char **argv) {
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

    Mat distanceCoefficients;

    Matrix4d tf_cam2base, tmpMat;
    Matrix3d rot4qua;
    vector<int> MarkerIds;

    vector<Matrix4d> tf_marker2base, tf_marker2cam;

    Quaterniond QVec;



    loadTransformationMatrix(tf_cam2base, packagePath + "/transformation_cam2base");





    //cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
    loadCameraCalibration(packagePath + "/IloveCameraCalibration", cameraMatrix, distanceCoefficients);


    ros::init(argc, argv, "destination_publisher");

    ros::NodeHandle n;

    ros::Publisher destination_publisher = n.advertise<cv_tracker::destination_msg>("helloworld/ef_destination", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    cv_tracker::destination_msg published_msg;
    while (ros::ok())
    {

        tf_marker2cam = startWebCamMonitoring(cameraMatrix, distanceCoefficients, 0.1245f, MarkerIds);
        for(int ind = 0; ind < MarkerIds.size(); ind++)
        {
            tmpMat =  tf_cam2base * tf_marker2cam[ind];

            Matrix4d test;
            test << 1,0,0,0,
                    0,0,-1,0,
                    0,1,0,0,
                    0,0,0,1;
            tmpMat = tmpMat*test;

            tf_marker2base.push_back(tmpMat);

            published_msg.frame_index = MarkerIds[ind];
            published_msg.x = tmpMat(0,3)-0.02;
            published_msg.y = tmpMat(1,3)+0.025;
            published_msg.z = tmpMat(2,3)-0.07;

            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    rot4qua(r,c) = tmpMat(r,c);
                }

            }
            Matrix3d test1;

//            test1 << 0.7071, 0, -0.7071,
//                          0, 1,       0,
//                     0.7071, 0,  0.7071;

            test1 << 0.7071, -0.7071, 0,
                     0.7071,  0.7071, 0,
                          0,       0, 1;


            rot4qua = rot4qua*test1;
            QVec = Quaterniond(rot4qua);

            published_msg.rx = QVec.x();
            published_msg.ry = QVec.y();
            published_msg.rz = QVec.z();
            published_msg.w = QVec.w();


            ROS_INFO_STREAM("No. " << MarkerIds[ind] << " [x,y,z]: [" << published_msg.x << ","
                                                           << published_msg.y << ","
                                                           << published_msg.z << "]\n"
                            << "Quaternion: [rx, ry, rz, w]: [" << published_msg.rx << ","
                                                             << published_msg.ry << ","
                                                             << published_msg.rz << "]\n");
            ROS_INFO_STREAM("No. " << MarkerIds[ind] << "marker wrt " << tf_marker2cam[ind](0,3)
                                                           << tf_marker2cam[ind](1,3) << tf_marker2cam[ind](2,3) << "\n");
            destination_publisher.publish(published_msg);
            count++;

        }

    }


    return 0;
}





