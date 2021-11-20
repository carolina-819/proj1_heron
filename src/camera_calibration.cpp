#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

int main(int argc, char **argv){

    std::vector<cv::String> filenames;
    cv::glob("calibration_Images/Images/*.jpg", filenames, false);
    cv::Size patternSize(10-1, 7-1);
    std::vector<std::vector<cv::Point2f>> q(filenames.size());

    std::vector<std::vector<cv::Point3f>> Q;

    int checkerBoard[2] = {10,7};
    
    std::vector<cv::Point3f>objp;
    for(int i=1; i<checkerBoard[1]; i++){
        for(int j=1; j<checkerBoard[0]; j++){
            objp.push_back(cv::Point3f(j,i,0));
        }
    }

    std::vector<cv::Point2f> imgPoint;

    std::size_t i = 0;
    for(auto const &f : filenames){
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(filenames[i]);
        cv::Mat gray;

        cv:cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK);
    
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }

        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("chessboard detection", img);
        cv::waitKey(0);

        i++;
    }
    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1280, 720);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;

    //#####################################Creates XML file########################################
    std::ofstream outfile("Matrix_coefficients.xml");
    outfile << "<camera_values>" << std::endl;
    outfile << "    <distortion_coefficients>" << std::endl;
    outfile << "        <k1>" << k(0) << "</k1>" << std::endl;
    outfile << "        <k2>" << k(1) << "</k2>" << std::endl;
    outfile << "        <p1>" << k(2) << "</p1>" << std::endl;
    outfile << "        <p2>" << k(3) << "</p2>" << std::endl;
    outfile << "        <k3>" << k(4) << "</k3>" << std::endl;
    outfile << "    </distortion_coefficients>" << std::endl;
    outfile << "    <camera_matrix>" << std::endl;
    outfile << "        " << K << std::endl;
    //outfile << "        <fx>" << K(0)(0) << "</fx>" << std::endl;
    //outfile << "        <fy>" << K(1)(1) << "</fy>" << std::endl;
    //outfile << "        <cx>" << K(0)(3) << "</cx>" << std::endl;
    //outfile << "        <cy>" << K(1)(3) << "</cy>" << std::endl;
    outfile << "    </camera_matrix>" << std::endl;
    outfile << "</camera_values>" << std::endl;

    //##############################################################################################


    outfile.close();

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

  // Show lens corrected images
  for (auto const &f : filenames) {
    std::cout << std::string(f) << std::endl;

    cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

    cv::Mat imgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

    // Display
    cv::imshow("undistorted image", imgUndistorted);
    cv::waitKey(0);
  }

  return 0;
}