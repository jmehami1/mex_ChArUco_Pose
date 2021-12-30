#include <string>
#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mex.h"
#include <vector>
#include <map>

#include "helper_functions.h"

#define NUM_INPUTS_MAT 7
#define NUM_OUTPUTS_MAT 3

/*
 * This is the function which estimates the pose of a ChArUco board all implemented in openCV
 *
*/
bool EstimateCharucoPose(cv::Mat &image, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &imageCopy,
                         const int numSquaresX, const int numSquaresY, const double checkerSideLength, const double arucoSideLength)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(numSquaresX, numSquaresY, checkerSideLength, arucoSideLength, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

    if (markerIds.size() > 0) {
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0){
            // if charuco pose is valid
            if (cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec)){
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                return true;
            }
        }
    }

    return false;
}


/*
 * Mex file which interfaces the Charuco code from opencv to allow it to be run in Matlab.
 * Inputs prhs[]:
 * [0] - uint8 input image (rows x cols x channels)
 * [1] - double camera intrinsic matrix [fx, 0, u0;
 *                                      0, fy, v0;
 *                                      0, 0, 1]
 * [2] - double camera distortion coefficients [K1, K2, P1, P2, K3]
 * [3] - uint8 [Number of squares in the X direction (Horizontal), Number of squares in the Y direction (vertical)]
 * [5] - double side length of checkerboard markers (metres)
 * [6] - double side length of Aruco marker (metres)
 *
 *SQUARES_X, SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH
 * Outputs plhs[]:
 * [0] - mxDouble rotation matrix [3 x 3]
 * [1] - mxDouble translation vector [tx, ty, tz]
 * [2] - mxLogical found pose of board
 * [3] - uint8 output image with drawn on axes only if board is found (rows x cols x channels)
*/

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{

    // Check for proper number of input arguments
    if (nrhs != NUM_INPUTS_MAT )
        mexErrMsgTxt("incorrect input arguments");


    //check for proper number of output arguments
    if (nlhs != NUM_OUTPUTS_MAT && nlhs != NUM_OUTPUTS_MAT+1)
        mexErrMsgTxt("incorrect output arguments");

    //**************1ST INPUT**************************

    cv::Mat inImgMat = mx_Array_Image2_Mat(prhs[0]);

    const mwSize *inImgDim = mxGetDimensions(prhs[0]);
    mwSize numImgDim = mxGetNumberOfDimensions(prhs[0]);

    //convert dimensions to integer
    const int inImgH = inImgDim[0];
    const int inImgW = inImgDim[1];

    if (inImgMat.empty())
        mexErrMsgTxt("Could not read in image to opencv MAT type");

    //**************2ND INPUT**************************

    const cv::Mat intrMatrix = double_mxArray_matrix2cv_Mat_matrix(prhs[1]);

    //**************3RD INPUT**************************

    const cv::Mat distCoef = double_mxArray_matrix2cv_Mat_matrix(prhs[2]);

    //**************4TH INPUT**************************

    const int numSquaresX = (int)mxGetScalar(prhs[3]);

    //    //**************5TH INPUT**************************

    const int numSquaresY = (int)mxGetScalar(prhs[4]);

    //    //**************6TH INPUT**************************

    const double checkerSideLength = (double)mxGetScalar(prhs[5]);

    //    //**************7TH INPUT**************************

    const double arucoSideLength = (double)mxGetScalar(prhs[6]);

    //Run the Charuco Pose estimation function
    cv::Vec3d rvec, tvec;

    //Copy passed in image
    cv::Mat imgWithPose;
    inImgMat.copyTo(imgWithPose);

    //get the pose of the board from the function
    bool foundPose = EstimateCharucoPose(inImgMat, intrMatrix, distCoef, rvec, tvec, imgWithPose, numSquaresX, numSquaresY, checkerSideLength, arucoSideLength);

    //Convert from Rodrigues vector to rotation matrix
    cv::Mat rotMat;
    cv::Rodrigues(rvec,rotMat);


    //**************1ST OUTPUT**************************

    plhs[0] = mxCreateDoubleMatrix(3,3, mxREAL);

    double* pr = mxGetPr(plhs[0]);

    int arrIndex = 0;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            *(pr+arrIndex) = rotMat.at<double>(j,i);
            arrIndex++;
        }
    }

    //**************SECOND OUTPUT**************************

    plhs[1] = mxCreateDoubleMatrix(1,3, mxREAL);

    pr = mxGetPr(plhs[1]);

    for (int i = 0; i < 3; i++)
        *(pr+i) = tvec[i];

    //**************THIRD OUTPUT**************************

    plhs[2] = mxCreateLogicalScalar(foundPose);

    //**************FOURTH OUTPUT************************** [OPTIONAL]

    if (nlhs == NUM_OUTPUTS_MAT+1) {
        plhs[3] = mxCreateNumericArray(numImgDim,inImgDim, mxUINT8_CLASS, mxREAL);

        char* outMat = (char*) mxGetData(plhs[3]);

        // grayscale image
        if (numImgDim == 2){
            arrIndex = 0;

            //Store image pixel channel colours into a 1D array used for passing to matlab
            for (int j = 0; j < inImgW; j++){
                for (int i = 0; i < inImgH; i++){
                    outMat[arrIndex] = imgWithPose.at<char>(i,j);

                    arrIndex++;
                }
            }
        }
        //RGB image
        else {
            cv::Vec3b pixel;
            arrIndex = 0;

            //Store image pixel channel colours into a 1D array used for passing to matlab
            for (int j = 0; j < inImgW; j++){
                for (int i = 0; i < inImgH; i++){
                    pixel = imgWithPose.at<cv::Vec3b>(i,j);

                    outMat[arrIndex] = pixel[2];   //R
                    outMat[inImgH*inImgW+arrIndex] = pixel[1]; //G
                    outMat[2*inImgH*inImgW+arrIndex] = pixel[0]; //B

                    arrIndex++;
                }
            }
        }
    }
}
