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



#define NUM_INPUTS_MAT 7
#define NUM_OUTPUTS_MAT 3

//#define SQUARES_X 8
//#define SQUARES_Y 6
//#define SQUARE_LENGTH 0.04f
//#define MARKER_LENGTH 0.03f


//template for creating opencv mat objects from input arrays of a specified data type
//template <typename T>
//cv::Mat createMatImage(T* data, int rows, int cols, int chs = 1)
//{
//    // Create Mat from buffer
//    cv::Mat mat(rows, cols, CV_MAKETYPE(cv::DataType<T>::type, chs));
//    memcpy(mat.data, data, rows*cols*chs * sizeof(T));
//    return mat;
//}


/*
 * This is the function which estimates the pose of a ChArUco board.
 *
*/
bool EstimateCharucoPose(cv::Mat &image, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,cv::Vec3d &rvec, cv::Vec3d &tvec, cv::Mat &imageCopy,
                         const int numSquaresX, const int numSquaresY, const double checkerSideLength, const double arucoSideLength)
{

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(numSquaresX, numSquaresY, checkerSideLength, arucoSideLength, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    params->doCornerRefinement = true;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

    if (markerIds.size() > 0) {
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0){

            //found = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);

            // if charuco pose is valid
            if (cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec)){
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));

                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                return true;
            }
        }
    } else {
        return false;
    }

    return false;
}


mwIndex subs(const mxArray *inputMatrix, const std::vector<mwIndex>& si)
{
    std::vector<mwIndex> v(si);
    return mxCalcSingleSubscript(inputMatrix, si.size(), (!v.empty() ? &v[0] : NULL));
}


//*mxArray Mat2_mx_Array(const cv::Mat& mat, mxClassID classid = mxUINT8_CLASS)
//{

//    // handle special case of empty input Mat by returning 0x0 array
//    if (mat.empty()) {
//        // TODO: maybe return empty array of same dimensions 0x1, 1x0x2, ...
//        p_ = mxCreateNumericMatrix(0, 0, classid, mxREAL);
//        if (!p_)
//            mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");
//        return;
//    }

//    // Create output mxArray (of specified type), equivalent to the input Mat
//    const mwSize cn = mat.channels();
//    const mwSize len = mat.total() * cn;
//    std::vector<mwSize> sz(mat.size.p, mat.size.p + mat.dims);
//    if (cn > 1)
//        sz.push_back(cn);  // channels is treated as another dimension
//    std::reverse(sz.begin(), sz.end());  // row vs. column major order
//    if (classid == mxLOGICAL_CLASS)
//        p_ = mxCreateLogicalArray(sz.size(), &sz[0]);
//    else
//        p_ = mxCreateNumericArray(sz.size(), &sz[0], classid, mxREAL);
//    if (!p_)
//        mexErrMsgIdAndTxt("mexopencv:error", "Allocation error");

//    // fill output with values from input Mat
//    // (linearized as a 1D-vector, both dimensions and channels)
//    {
//        // wrap destination data using a cv::Mat
//        const int type = CV_MAKETYPE(DepthOf[classid], 1); // destination type
//        cv::Mat m(len, 1, type, mxGetData(p_));  // only creates Mat header

//        // copy flattened input to output array (converting to specified type)
//        const cv::Mat mat0(len, 1, mat.depth(), mat.data); // no data copying
//        if (classid == mxLOGICAL_CLASS) {
//            // OpenCV's logical true is any nonzero, while MATLAB's true is 1
//            cv::compare(mat0, 0, m, cv::CMP_NE); // values either 0 or 255
//            m.setTo(1, m);  // values either 0 or 1 (CV_8U)
//        }
//        else
//            mat0.convertTo(m, type);
//    }

//    // rearrange dimensions of mxArray by calling PERMUTE from MATLAB. We want
//    // to convert from row-major order (C-style, last dim changes fastest) to a
//    // column-major order (MATLAB-style, first dim changes fastest). This will
//    // handle all cases of cv::Mat as multi-channels and/or multi-dimensions.
//    std::vector<double> order;
//    order.reserve(sz.size());
//    for (int i=sz.size(); i>0; i--)
//        order.push_back(i);

//    // CALL: out = permute(in, ndims(in):-1:1)
//    mxArray *lhs, *rhs[2];
//    rhs[0] = const_cast<mxArray*>(p_);
//    rhs[1] = MxArray(order);
//    lhs = NULL;              // new data copy will be returned
//    if (mexCallMATLAB(1, &lhs, 2, rhs, "permute") != 0)
//        mexErrMsgIdAndTxt("mexopencv:error", "Error calling permute");
//    p_ = lhs;
//    mxDestroyArray(rhs[0]);  // discard old copy
//    mxDestroyArray(rhs[1]);
//    CV_DbgAssert(!isNull() && classID()==classid && numel()==len);
//}




cv::Mat mx_Array_Image2_Mat(const mxArray *inputMatrix)//, int depth, bool transpose)
{

    uint8_T *inImgArr = mxGetUint8s(inputMatrix);
    const mwSize *dims = mxGetDimensions(inputMatrix);
    mwSize ndims = mxGetNumberOfDimensions(inputMatrix);

    const mxClassID classID = mxGetClassID(inputMatrix);



    // Create cv::Mat object (of the specified depth), equivalent to mxArray.
    // At this point we create either a 2-dim with 1-channel mat, or a 2-dim
    // with multi-channels mat. Multi-dims case is handled above.
    std::vector<int> d(dims, dims+ndims);
    ndims = (d.size()>2) ? d.size()-1 : d.size();
    const mwSize nchannels = (d.size()>2) ? d.back() : 1;
    int depth = CV_8U;
    std::swap(d[0], d[1]);
    cv::Mat mat(ndims, &d[0], CV_MAKETYPE(depth, nchannels));
    // Copy each channel from mxArray to Mat (converting to specified depth),
    // as in: channels[i] <- cast_to_mat_depth(p_(:,:,i))
    std::vector<cv::Mat> channels(nchannels);
    std::vector<mwSize> si(d.size(), 0);                 // subscript index
    const int type = CV_MAKETYPE(depth, 1); // Source type
    for (mwIndex i = 0; i<nchannels; ++i) {
        si[si.size() - 1] = i;                   // last dim is a channel idx
        void *pd = reinterpret_cast<void*>(
                    reinterpret_cast<size_t>(mxGetData(inputMatrix)) +
                    mxGetElementSize(inputMatrix)*subs(inputMatrix ,si));      // ptr to i-th channel data
        const cv::Mat m(ndims, &d[0], type, pd); // only creates Mat headers
        // Read from mxArray through m, writing into channels[i]
        // (Note that saturate_cast<> is applied, so values are clipped
        // rather than wrap-around in a two's complement sense. In
        // floating-point to integer conversion, numbers are first rounded
        // to nearest integer then clamped).
        m.convertTo(channels[i], CV_MAKETYPE(depth, 1));
        // transpose cv::Mat if needed. We do this inside the loop on each 2d
        // 1-cn slice to avoid cv::transpose limitation on number of channels
        //if (transpose)
        cv::transpose(channels[i], channels[i]);  // in-place transpose
    }
    // Merge channels back into one cv::Mat array
    cv::merge(channels, mat);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    //const cv::Mat mat = (cv::Mat_<float>(1,5) << 0.0347, -0.0918, -0.0042, 0.0057, -0.0898);
    return mat;
}

/*
 * Converts a mxArray containing a double type matrix to an opencv Mat double matrix
*/
cv::Mat double_mxArray_matrix2cv_Mat_matrix(const mxArray *inputMatrix)
{
    double* inDoubleArr = mxGetDoubles(inputMatrix);
    const mwSize *inDim = mxGetDimensions(inputMatrix);

    //Size of array from Matlab
    const int rows = inDim[0];
    const int cols = inDim[1];

    //Create Mat_ intermediate storage
    cv::Mat_<double> M(rows,cols);

    int indexArr = 0;

    //put all data into the M matrix
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            M[i][j] = *(inDoubleArr + indexArr);
            indexArr++;
        }
    }

    return ((cv::Mat) M);
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

    //get pointer to passed in image from Matlab.

    cv::Mat inImgMat = mx_Array_Image2_Mat(prhs[0]);

    //std::cout << "M = " << std::endl << " "  << inImgMat << std::endl << std::endl;

    //uint8_T *inImgArr = mxGetUint8s(prhs[0]);
    const mwSize *inImgDim = mxGetDimensions(prhs[0]);
    mwSize numImgDim = mxGetNumberOfDimensions(prhs[0]);

    //convert dimensions to integer
    const int inImgH = inImgDim[0];
    const int inImgW = inImgDim[1];
    const int inImgC = inImgDim[2];


    //    //Need to reorganise the array to correctly be read into opencv MAT object
    ////    uint8_T inImgArrOrg[inImgH*inImgW*inImgC] = {0};

    ////    for (int i = 0; i < inImgH*inImgW; i++){
    ////        for (int j = 0; j < inImgC; j++)
    ////            inImgArrOrg[i*inImgC+j] = inImgArr[j*inImgH*inImgW + i];
    ////    }

    //    //Create opencv MAT type
    //    cv::Mat inImgMat = createMatImage<uint8_T>(inImgArr, inImgH, inImgW, inImgC);


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


    //     //uint8_T *pt = mxGetUint8s(prhs[3]);
    //    //const int numSquaresX = pt[0];

    //     std::cout << numSquaresX << " " << numSquaresY << " " << checkerSideLength << " " std::endl;



    //Run the Charuco Pose estimation function
    cv::Vec3d rvec, tvec;
    //bool foundPose = false;

    //    const cv::Mat intrMatrix = (cv::Mat_<float>(3,3) << 532.5681, 0, 327.4995, 0, 531.9054, 231.2278, 0, 0, 1);
    //    const cv::Mat distCoef = (cv::Mat_<float>(1,5) << 0.0347, -0.0918, -0.0042, 0.0057, -0.0898);

    cv::Mat imgWithPose;
    inImgMat.copyTo(imgWithPose);

    cv::imwrite("input.png", inImgMat);


    bool foundPose = EstimateCharucoPose(inImgMat, intrMatrix, distCoef, rvec, tvec, imgWithPose, numSquaresX, numSquaresY, checkerSideLength, arucoSideLength);

    cv::imwrite("output.png", imgWithPose);

    //    std::cout << rvec << std::endl;

    //Convert from Rodrigues vector to rotation matrix
    cv::Mat rotMat;
    cv::Rodrigues(rvec,rotMat);


    //    //**************FIRST OUTPUT**************************

    plhs[0] = mxCreateDoubleMatrix(3,3, mxREAL);

    double* pr = mxGetPr(plhs[0]);

    int arrIndex = 0;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            //            pr[arrIndex] = rotMat.at<double>(j,i);
            *(pr+arrIndex) = rotMat.at<double>(j,i);
            arrIndex++;
        }
    }

    //**************SECOND OUTPUT**************************

    plhs[1] = mxCreateDoubleMatrix(1,3, mxREAL);

    pr = mxGetPr(plhs[1]);

    for (int i = 0; i < 3; i++)
        *(pr+i) = tvec[i];

    //        std::cout << "FLAG 2" << std::endl;


    //    //**************THIRD OUTPUT**************************

    plhs[2] = mxCreateLogicalScalar(foundPose);

           std::cout << "FLAG 3" << std::endl;


    //**************FOURTH OUTPUT************************** [OPTIONAL]

    if (nlhs == NUM_OUTPUTS_MAT+1) {

        plhs[3] = mxCreateNumericArray(numImgDim,inImgDim, mxUINT8_CLASS, mxREAL);

        char* outMat = (char*) mxGetData(plhs[3]);

        cv::Vec3b pixel;
        arrIndex = 0;

        //for (int k = 0; k < inImgC; k++){
        for (int j = 0; j < inImgW; j++){
            for (int i = 0; i < inImgH; i++){
                pixel = imgWithPose.at<cv::Vec3b>(i,j);

                outMat[arrIndex] = pixel[2];   //R
                outMat[inImgH*inImgW+arrIndex] = pixel[1]; //G
                outMat[2*inImgH*inImgW+arrIndex] = pixel[0]; //B

                //(outMat[arrIndex] = imgWithPose.at<uchar>(i,j,k);

                arrIndex++;
            }
        }
       // }
    }

    //    std::cout << "FLAG 4" << std::endl;

}
