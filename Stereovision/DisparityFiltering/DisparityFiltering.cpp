#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);

typedef void* MonoArray;

enum { kMonoArrayOffset = 16 };
template<class T> T GetMonoArrayElement(MonoArray* array, int i) {
    char* raw = kMonoArrayOffset + i * sizeof(T) + (char*)array;
    return *(T*)raw;
}
template<class T> void SetMonoArrayElement(MonoArray* array, int i, T elem) {
    char* raw = kMonoArrayOffset + i * sizeof(T) + (char*)array;
    *(T*)raw= elem;
}

int main(int argc, char** argv) {
    String algo = "bm";
    String filter = "wls_conf";
    bool no_display = true;
    bool no_downscale = true;
    int max_disp = 160;
    double lambda = 8000.0;
    double sigma  = 1.5;
    double fbs_spatial = 16.0;
    double fbs_luma = 8.0;
    double fbs_chroma = 8.0;
    double fbs_lambda = 128.0;
    double vis_mult = 1.0;

    int wsize;
    if(algo=="sgbm")
        wsize = 3; //default window size for SGBM
    else if(!no_downscale && algo=="bm" && filter=="wls_conf")
        wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
    else
        wsize = 15; //default window size for BM on full-sized views


    Mat left  = cv::imread("../../../SV_In/stLeft.jpg" ,IMREAD_GRAYSCALE);
    if ( left.empty() )
    {
        //std::cout<<"Cannot read image file: "<<left_im;
        return -1;
    }
    cv::resize(left, left, cv::Size(left.cols * 0.25,left.rows * 0.25), 0, 0, INTER_LINEAR_EXACT);
    
    Mat right = cv::imread("../../../SV_In/stRight.jpg",IMREAD_GRAYSCALE);
    if ( right.empty() )
    {
        //std::cout<<"Cannot read image file: "<<right_im;
        return -1;
    }
    cv::resize(right, right, cv::Size(right.cols * 0.25,right.rows * 0.25), 0, 0, INTER_LINEAR_EXACT);

    Mat rectifiedLeft;
    Mat rectifiedRight;
    Mat Q;

    // Load camera matrices and distortion coefficients from calibration
    cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R, T;
    cv::FileStorage fs("../../../calibration_params/stereocalib_params.xml", cv::FileStorage::READ);
    fs["cameraMatrix1"] >> cameraMatrix1;
    fs["cameraMatrix2"] >> cameraMatrix2;
    fs["distCoeffs1"] >> distCoeffs1;
    fs["distCoeffs2"] >> distCoeffs2;
    fs["rotationVectors"] >> R;
    fs["translationVectors"] >> T;
    fs.release();

    // Image size
    cv::Size imageSize = left.size();

    // Rectification transformation matrices
    cv::Mat R1, R2, P1, P2;

    // Perform stereo rectification
    cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q);

    // Use the rectification matrices to rectify the images
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, map21, map22);

    cv::remap(left, rectifiedLeft, map11, map12, cv::INTER_LINEAR);
    cv::remap(right, rectifiedRight, map21, map22, cv::INTER_LINEAR);

    cv::namedWindow("rectified image", WINDOW_AUTOSIZE);
    cv::imshow("rectified image", rectifiedLeft);

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp,solved_disp,solved_filtered_disp;
    Mat conf_map = Mat(2464 / 4,3280 / 4,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;
    double solving_time = 0;
    if(max_disp<=0 || max_disp%16!=0)
    {
        std::cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
        return -1;
    }
    if(wsize<=0 || wsize%2!=1)
    {
        std::cout<<"Incorrect window_size value: it should be positive and odd";
        return -1;
    }

    if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
    {
        if(!no_downscale)
        {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale]
            max_disp/=2;
            if(max_disp%16!=0)
                max_disp += 16-(max_disp%16);
            cv::resize(rectifiedLeft ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
            cv::resize(rectifiedRight,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
            //! [downscale]
        }
        else
        {
            left_for_matcher  = rectifiedLeft.clone();
            right_for_matcher = rectifiedRight.clone();
        }
        if(algo=="bm")
        {
            //! [matching]
            Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            //cv::cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY); Only necessary if images are color (BGR), currently grayscale
            //cv::cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            //! [matching]
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
            left_matcher->setP1(24*wsize*wsize);
            left_matcher->setP2(96*wsize*wsize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            std::cout<<"Unsupported algorithm";
            return -1;
        }

        //! [filtering]
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        filtering_time = (double)getTickCount();
        wls_filter->filter(left_disp,rectifiedLeft,filtered_disp,right_disp);
        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        //! [filtering]
        conf_map = wls_filter->getConfidenceMap();

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            cv::resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
            left_disp = left_disp*2.0;
            ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }
    }
    else if(filter=="fbs_conf") // filtering with fbs and confidence using also wls pre-processing
    {
        if(!no_downscale)
        {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale_wls]
            max_disp/=2;
            if(max_disp%16!=0)
                max_disp += 16-(max_disp%16);
            cv::resize(rectifiedLeft ,left_for_matcher ,Size(),0.5,0.5);
            cv::resize(rectifiedRight,right_for_matcher,Size(),0.5,0.5);
            //! [downscale_wls]
        }
        else
        {
            left_for_matcher  = rectifiedLeft.clone();
            right_for_matcher = rectifiedRight.clone();
        }

        if(algo=="bm")
        {
            //! [matching_wls]
            Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            //cv::cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
            //cv::cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            //! [matching_wls]
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
            left_matcher->setP1(24*wsize*wsize);
            left_matcher->setP2(96*wsize*wsize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            std::cout<<"Unsupported algorithm";
            return -1;
        }

        //! [filtering_wls]
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        filtering_time = (double)getTickCount();
        wls_filter->filter(left_disp,rectifiedLeft,filtered_disp,right_disp);
        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        //! [filtering_wls]

        conf_map = wls_filter->getConfidenceMap();

        Mat left_disp_resized;
        cv::resize(left_disp,left_disp_resized,rectifiedLeft.size());

        // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            cv::resize(left_disp,left_disp,Size(),2.0,2.0);
            left_disp = left_disp*2.0;
            left_disp_resized = left_disp_resized*2.0;
            ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }

#ifdef HAVE_EIGEN
        //! [filtering_fbs]
        solving_time = (double)getTickCount();
        fastBilateralSolverFilter(rectifiedLeft, left_disp_resized, conf_map/255.0f, solved_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
        solving_time = ((double)getTickCount() - solving_time)/getTickFrequency();
        //! [filtering_fbs]

        //! [filtering_wls2fbs]
        fastBilateralSolverFilter(rectifiedLeft, filtered_disp, conf_map/255.0f, solved_filtered_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
        //! [filtering_wls2fbs]
#else
        (void)fbs_spatial;
        (void)fbs_luma;
        (void)fbs_chroma;
        (void)fbs_lambda;
#endif
    }
    else if(filter=="wls_no_conf")
    {
        /* There is no convenience function for the case of filtering with no confidence, so we
        will need to set the ROI and matcher parameters manually */

        left_for_matcher  = rectifiedLeft.clone();
        right_for_matcher = rectifiedRight.clone();

        if(algo=="bm")
        {
            Ptr<StereoBM> matcher  = StereoBM::create(max_disp,wsize);
            matcher->setTextureThreshold(0);
            matcher->setUniquenessRatio(0);
            //cv::cvtColor(left_for_matcher,  left_for_matcher, COLOR_BGR2GRAY);
            //cv::cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> matcher  = StereoSGBM::create(0,max_disp,wsize);
            matcher->setUniquenessRatio(0);
            matcher->setDisp12MaxDiff(1000000);
            matcher->setSpeckleWindowSize(0);
            matcher->setP1(24*wsize*wsize);
            matcher->setP2(96*wsize*wsize);
            matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            ROI = computeROI(left_for_matcher.size(),matcher);
            wls_filter = createDisparityWLSFilterGeneric(false);
            wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            std::cout<<"Unsupported algorithm";
            return -1;
        }

        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        filtering_time = (double)getTickCount();
        wls_filter->filter(left_disp,rectifiedLeft,filtered_disp,Mat(),ROI);
        filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    }
    else
    {
        std::cout<<"Unsupported filter";
        return -1;
    }

    //collect and print all the stats:
    std::cout.precision(2);
    std::cout<<"Matching time:  "<<matching_time<<"s"<<endl;
    std::cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
    std::cout<<"Solving time: "<<solving_time<<"s"<<endl;
    std::cout<<endl;
    

    Mat filtered_disp_vis;
    cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);

    cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    cv::imshow("filtered disparity", filtered_disp_vis);
    cv::waitKey(0);

    return 1;
}

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

extern "C" int processDisparity(MonoArray* pointCloudData, int height, int width, MonoArray* leftImage, MonoArray* rightImage) {
    String algo = "bm";
    String filter = "wls_conf";
    bool no_display = true;
    bool no_downscale = true;
    int max_disp = 160;
    double lambda = 8000.0;
    double sigma  = 1.5;
    double fbs_spatial = 16.0;
    double fbs_luma = 8.0;
    double fbs_chroma = 8.0;
    double fbs_lambda = 128.0;
    double vis_mult = 1.0;

    int wsize;
    if(algo=="sgbm")
        wsize = 3; //default window size for SGBM
    else if(!no_downscale && algo=="bm" && filter=="wls_conf")
        wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
    else
        wsize = 15; //default window size for BM on full-sized views

    Mat left(height,width,CV_8U);
    uchar* leftMatData = left.ptr<uchar>(); // Get pointer to the start of the data buffer
    int numElements = height * width;
    for (int i = 0; i < numElements; i++) {
        leftMatData[i] = (uchar) GetMonoArrayElement<int>(leftImage, i);
    }


    if ( left.empty() )
    {
        //std::cout<<"Cannot read image file: "<<left_im;
        return -1;
    }

    Mat right(height,width,CV_8U);
    uchar* rightMatData = right.ptr<uchar>(); // Get pointer to the start of the data buffer
    for (int i = 0; i < numElements; i++) {
        rightMatData[i] = (uchar) GetMonoArrayElement<int>(rightImage, i);
    }

    if ( right.empty() )
    {
        //std::cout<<"Cannot read image file: "<<right_im;
        return -1;
    }

    Mat rectifiedLeft;
    Mat rectifiedRight;
    Mat Q;

    // Load camera matrices and distortion coefficients from calibration
    cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R, T;
    cv::FileStorage fs("../calibration_params/stereocalib_params.xml", cv::FileStorage::READ);
    fs["cameraMatrix1"] >> cameraMatrix1;
    fs["cameraMatrix2"] >> cameraMatrix2;
    fs["distCoeffs1"] >> distCoeffs1;
    fs["distCoeffs2"] >> distCoeffs2;
    fs["rotationVectors"] >> R;
    fs["translationVectors"] >> T;
    fs.release();

    // Image size
    cv::Size imageSize = left.size();

    // Rectification transformation matrices
    cv::Mat R1, R2, P1, P2;

    // Perform stereo rectification
    cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q);

    // Use the rectification matrices to rectify the images
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_16SC2, map21, map22);

    cv::remap(left, rectifiedLeft, map11, map12, cv::INTER_LINEAR);
    cv::remap(right, rectifiedRight, map21, map22, cv::INTER_LINEAR);


    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp,solved_disp,solved_filtered_disp;
    Mat conf_map = Mat(height,width,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;
    double solving_time = 0;
    if(max_disp<=0 || max_disp%16!=0)
    {
        std::cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
        return -1;
    }
    if(wsize<=0 || wsize%2!=1)
    {
        std::cout<<"Incorrect window_size value: it should be positive and odd";
        return -1;
    }

    if(!no_downscale)
    {
        // downscale the views to speed-up the matching stage, as we will need to compute both left
        // and right disparity maps for confidence map computation
        //! [downscale]
        max_disp/=2;
        if(max_disp%16!=0)
            max_disp += 16-(max_disp%16);
        cv::resize(rectifiedLeft ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
        cv::resize(rectifiedRight,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
        //! [downscale]
    }
    else
    {
        left_for_matcher  = rectifiedLeft.clone();
        right_for_matcher = rectifiedRight.clone();
    }
    if(algo=="bm")
    {
        //! [matching]
        Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
        wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        //cv::cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY); Only necessary if images are color (BGR), currently grayscale
        //cv::cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

        matching_time = (double)getTickCount();
        left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
        matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        //! [matching]
    }
    else if(algo=="sgbm")
    {
        Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
        left_matcher->setP1(24*wsize*wsize);
        left_matcher->setP2(96*wsize*wsize);
        left_matcher->setPreFilterCap(63);
        left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
        wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        matching_time = (double)getTickCount();
        left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
        matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
    }
    else
    {
        std::cout<<"Unsupported algorithm";
        return -1;
    }

    //! [filtering]
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,rectifiedLeft,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    //! [filtering]
    conf_map = wls_filter->getConfidenceMap();

    // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();
    if(!no_downscale)
    {
        // upscale raw disparity and ROI back for a proper comparison:
        cv::resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
        left_disp = left_disp*2.0;
        ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
    }
    //collect and print all the stats:
    std::cout.precision(2);
    std::cout<<"Matching time:  "<<matching_time<<"s"<<endl;
    std::cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
    std::cout<<"Solving time: "<<solving_time<<"s"<<endl;
    std::cout<<endl;
    

    Mat filtered_disp_vis;
    cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);

    // Reproject to 3D Space
    Mat pointCloud;
    cv::reprojectImageTo3D(filtered_disp_vis, pointCloud, Q, true);

    for (int i = 0; i < pointCloud.rows; i++)
    {
        for (int j = 0; j < pointCloud.cols; j++)
        {
            SetMonoArrayElement<float> (pointCloudData, i * (pointCloud.cols * 3) + j * 3, pointCloud.at<Vec3f>(i, j)[1]);
            SetMonoArrayElement<float> (pointCloudData, i * (pointCloud.cols * 3) + j * 3 + 1, pointCloud.at<Vec3f>(i, j)[2]);
            SetMonoArrayElement<float> (pointCloudData, i * (pointCloud.cols * 3) + j * 3 + 2, pointCloud.at<Vec3f>(i, j)[0]);
        }
    }

    return 1;
}