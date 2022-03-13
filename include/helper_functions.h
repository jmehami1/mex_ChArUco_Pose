#include <opencv2/imgproc/imgproc.hpp>
#include "mex.h"
#include <vector>
//#include <map>
//#include <yaml-cpp/yaml.h>



//Helper function for used in mx_Array_Image2_Mat
mwIndex subs(const mxArray *inputMatrix, const std::vector<mwIndex>& si);

cv::Mat mx_Array_Image2_Mat(const mxArray *inputMatrix);
/*
 * Converts a mxArray containing a double type matrix to an opencv Mat double matrix
*/
cv::Mat double_mxArray_matrix2cv_Mat_matrix(const mxArray *inputMatrix);

//void IntVector2mxIntRowArray(mxArray *mxRowArr, std::vector<int> inVec);
/*
 * Read the ArUco board parameters from a yaml file. This file should contain the IDs present on the board and their respective corners.
 *
 */
//bool Read_ArUco_YAML(const std::string fileName, cv::Ptr<cv::aruco::Dictionary> dictionary, std::vector<int> ids, std::vector<std::vector<cv::Point3f>>objPoints)
//{
//    YAML::Node yanlReader = YAML::LoadFile(fileName);

//}
