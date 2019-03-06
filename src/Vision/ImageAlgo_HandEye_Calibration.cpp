/*
 * Author: Yu-Shiang Yan
 *
 * Note: Eye in hand calibration interface(Ex. camera calibration, hand eye calibration)
 */

#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <vector>
#include <math.h>
#include <fstream>
#include <stdlib.h>

#include <Image_Params.h>
#include <ImageAlgo_HandEye_Calibration.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#define dVAMODEL

#ifndef dVAMODEL
/* 4 axis param */
#define dGRID_SIZE 5
double cameraWidth = 64.532;
double cameraHeight = 0;
#define CR_IGNORE_Z_VECTOR
#else
/* 6 axis param */
// using ruler to measure
//double cameraWidth = 50;
//double cameraTheta = 45;
//double cameraHeight = 25;
#endif

#define CV_CALIBRATION_TSAI
//#define CV_ROTATION_DEBUG

using namespace cv;

static std::vector<Point2f> CRAlgoSortCornerPoints(std::vector<Point2f> corners)
{
    int i, num;

    std::vector<Point2f> output;

    num = corners.size();

    if((corners[num - 1].x < corners[0].x)
        && (corners[num - 1].y < corners[0].y))
    {
        for(i = 0; i < num; i++)
        {
            output.push_back(corners[(num-1)-i]);
        }
    }
    else
    {
        for(i = 0; i < num; i++)
        {
            output.push_back(corners[i]);
        }
    }

    return output;
}


static std::vector<std::vector<Point3f> > CRAlgoCalcBoardCornerPositionsList(int gridW, int gridH, double squareSize, int imagesCount)
{
  std::vector<std::vector<Point3f> > objectPointsList(imagesCount);
    for (int k = 0; k <imagesCount; k++) {
        objectPointsList[k] = std::vector<Point3f>(0);
        for (int i = 0; i < gridH; i++)
            for (int j = 0; j < gridW; j++)
                objectPointsList[k].push_back(Point3f(double(j*squareSize), double(i*squareSize), 0));
    }
    return objectPointsList;
}

static Mat CRSkewVec(Mat vec)
{
    Mat resultMat(3, 3, CV_64FC1);

    resultMat.at<double>(0,0) = 0;
    resultMat.at<double>(0,1) = -vec.at<double>(2,0);
    resultMat.at<double>(0,2) = vec.at<double>(1,0);
    resultMat.at<double>(1,0) = vec.at<double>(2,0);
    resultMat.at<double>(1,1) = 0;
    resultMat.at<double>(1,2) = -vec.at<double>(0,0);
    resultMat.at<double>(2,0) = -vec.at<double>(1,0);
    resultMat.at<double>(2,1) = vec.at<double>(0,0);
    resultMat.at<double>(2,2) = 0;

    return resultMat;
}

static void CRTsaiHandEye(Mat& eMc, std::vector<Mat> Hgij, std::vector<Mat> Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);

    Mat rgij(3, 1, CV_64FC1);
    Mat rcij(3, 1, CV_64FC1);

    double theta_gij;
    double theta_cij;

    Mat rngij(3, 1, CV_64FC1);
    Mat rncij(3, 1, CV_64FC1);

    Mat Pgij(3, 1, CV_64FC1);
    Mat Pcij(3, 1, CV_64FC1);
    Mat PSumij(3, 1, CV_64FC1);

    Mat tempA(3, 3, CV_64FC1);
    Mat tempb(3, 1, CV_64FC1);

    Mat A;
    Mat b;
    Mat pinA;

    Mat Pcg_prime(3, 1, CV_64FC1);
    Mat Pcg(3, 1, CV_64FC1);
    Mat PcgTrs(1, 3, CV_64FC1);

    Mat Rcg(3, 3, CV_64FC1);
    Mat eyeM = Mat::eye(3, 3, CV_64FC1);

    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat tempAA(3, 3, CV_64FC1);
    Mat tempbb(3, 1, CV_64FC1);

    Mat AA;
    Mat bb;
    Mat pinAA;

    Mat Tcg(3, 1, CV_64FC1);

    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

        Rodrigues(Rgij, rgij);
        Rodrigues(Rcij, rcij);

        theta_gij = norm(rgij);
        theta_cij = norm(rcij);

        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;

        Pgij = 2 * sin(theta_gij / 2)*rngij;
        Pcij = 2 * sin(theta_cij / 2)*rncij;

        PSumij = (Pgij + Pcij);

        tempA = CRSkewVec(PSumij);
        tempb = Pcij - Pgij;


        //cout <<  "tempA:" << endl << tempA << endl;
        //cout <<  "tempb:" << endl << tempb << endl;

        A.push_back(tempA);
        b.push_back(tempb);
    }

    //Compute rotation
    invert(A, pinA, DECOMP_SVD);

    Pcg_prime = pinA * b;
    Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
    PcgTrs = Pcg.t();
    Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*CRSkewVec(Pcg));

    //Computer Translation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);


        tempAA = Rgij - eyeM;
        tempbb = Rcg * Tcij - Tgij;

        //cout << "Tcij:" << Tcij << endl;
        //cout << "Tgij:" << Tgij << endl;

        AA.push_back(tempAA);
        bb.push_back(tempbb);
    }

    invert(AA, pinAA, DECOMP_SVD);
    Tcg = pinAA * bb;

    //cout << "bb:" << endl << bb << endl;
    //cout << "R:" << endl << Rcg << endl;
    //cout << "T:" << endl << Tcg << endl;

    Rcg.copyTo(eMc(Rect(0, 0, 3, 3)));
    Tcg.copyTo(eMc(Rect(3, 0, 1, 3)));
    eMc.at<double>(3, 0) = 0.0;
    eMc.at<double>(3, 1) = 0.0;
    eMc.at<double>(3, 2) = 0.0;
    eMc.at<double>(3, 3) = 1.0;
}

int CRAlgoTestChessBoard(Mat             		srcImg,
                         std::vector<Point2f>*	ImageCorners)
{
    int exitCode = dFALSE,
        i;

    std::vector<Point2f> corners,
                         sortingCorners;

    bool found = false;

    Size grids(dGRID_WIDTH_OF_NUM,dGRID_HEIGHT_OF_NUM); //number of centers

    try
    {
        found = findChessboardCorners(srcImg, grids, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (found)
        {
            Mat     gray;

            cvtColor(srcImg, gray, COLOR_BGR2GRAY);

            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));//0.1 is accuracy

            gray.release();

            sortingCorners = CRAlgoSortCornerPoints(corners);

            for(i = 0; i < (int)sortingCorners.size(); i++)
            {
                ImageCorners->push_back(sortingCorners[i]);
            }

            std::vector<Point2f>().swap(corners);

        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}
int CRAlgoCalcHomogeneousMat(Mat     translateVec,
                             Mat     rotationMat,
                             Mat&    homogeneousMat)
{
    int exitCode = dFALSE,
        i        = 0,
        j        = 0;

    try
    {
        for(j = 0; j < 3; j++)
        {
            for(i = 0; i < 3; i++)
            {
                homogeneousMat.at<double>(i,j) = (double)rotationMat.at<double>(i,j);
            }

            homogeneousMat.at<double>(j,3) = (double)translateVec.at<double>(j,0);

            homogeneousMat.at<double>(3,j) = 0.0;
        }

        homogeneousMat.at<double>(3,3) = 1;
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoCalcAngleToRmat(CRAngleInfo 	angle,
                           Mat&        	R)
{
    int exitCode = dFALSE;

    double tempXTheta = 0.0,
           tempYTheta = 0.0,
           tempZTheta = 0.0;
    try
    {
        Mat tempRx(3, 3, CV_64FC1),
            tempRy(3, 3, CV_64FC1),
            tempRz(3, 3, CV_64FC1),
            tempR;

        tempXTheta = (angle.rx  * dPI)/180;
        tempYTheta = (angle.ry  * dPI)/180;
        tempZTheta = (angle.rz  * dPI)/180;

        tempRx.at<double>(0,0) = 1;
        tempRx.at<double>(0,1) = 0;
        tempRx.at<double>(0,2) = 0;
        tempRx.at<double>(1,0) = 0;
        tempRx.at<double>(1,1) = cos(tempXTheta);
        tempRx.at<double>(1,2) = -sin(tempXTheta);
        tempRx.at<double>(2,0) = 0;
        tempRx.at<double>(2,1) = sin(tempXTheta);
        tempRx.at<double>(2,2) = cos(tempXTheta);

        tempRy.at<double>(0,0) = cos(tempYTheta);
        tempRy.at<double>(0,1) = 0;
        tempRy.at<double>(0,2) = sin(tempYTheta);
        tempRy.at<double>(1,0) = 0;
        tempRy.at<double>(1,1) = 1;
        tempRy.at<double>(1,2) = 0;
        tempRy.at<double>(2,0) = -sin(tempYTheta);
        tempRy.at<double>(2,1) = 0;
        tempRy.at<double>(2,2) = cos(tempYTheta);

        tempRz.at<double>(0,0) = cos(tempZTheta);
        tempRz.at<double>(0,1) = -sin(tempZTheta);
        tempRz.at<double>(0,2) = 0;
        tempRz.at<double>(1,0) = sin(tempZTheta);
        tempRz.at<double>(1,1) = cos(tempZTheta);
        tempRz.at<double>(1,2) = 0;
        tempRz.at<double>(2,0) = 0;
        tempRz.at<double>(2,1) = 0;
        tempRz.at<double>(2,2) = 1;

        tempR = tempRz * tempRy * tempRx;
        tempR.copyTo(R);

        tempRz.release();
        tempRy.release();
        tempRx.release();
        tempR.release();
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoCalcRmatToAngle(Mat 			R,
                          CRAngleInfo& 	angle)
{
    int exitCode = dFALSE;

    double rx = 0.0f,
           ry = 0.0f,
           rz = 0.0f;

    try
    {
        // paper's Algo RzRyRx
        if(R.at<double>(2,0) < 1)
        {
            if(R.at<double>(2,0) > -1)
            {
                ry = asin(-R.at<double>(2,0)) ;
                rz = atan2(R.at<double>(1,0) , R.at<double>(0,0)) ;
                rx = atan2(R.at<double>(2,1) , R.at<double>(2,2)) ;
            }
            else //r20 = -1
            {
                // Not a unique solution: X - Z = atan2(-r12 , r11)
                ry = dPI/2 ;
                rz = -atan2(-R.at<double>(1,2) , R.at<double>(1,1)) ;
                rx = 0 ;
            }
        }
        else //r20 = 1
        {
            // Not a unique solution: the X + Z = atan2(-r12 , r11)
            ry = -dPI/2 ;
            rz = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            rx = 0 ;
        }

        rx = (rx * 180)/dPI;
        ry = (ry * 180)/dPI;
        rz = (rz * 180)/dPI;

        angle.rx = rx;
        angle.ry = ry;
        angle.rz = rz;

    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoCVSaveResultAsImg(unsigned char*    raw_data,
              int               img_w,
              int               img_h,
              int               img_pitch,
              char*             img_str,
              float			  rx,
              float 			  ry)
{
   int             exitCode = dFALSE;
   std::string          str(img_str);

   try
   {

     int pX, pY, tmpX, tmpY, c;
     pX = (int)rx;
     pY = (int)ry;

     for (c = -10; c < 11; c++)
     {
       tmpX = pX + c;
       tmpY = pY + c;

       if((tmpX > img_w-1) || (tmpY > img_h-1) || (tmpX < 0) || (tmpY < 0))
       {
         continue;
       }

       raw_data[(tmpY) * img_pitch + (pX)] = 150;
       raw_data[(pY) * img_pitch + (tmpX)] = 150;
     }

     // create Mat
       Mat image(Size(img_w, img_h), CV_8UC1, raw_data, Mat::AUTO_STEP);
       // save as image
       imwrite(str, image);
   }
   catch( cv::Exception& e )
   {
       const char* err_msg = e.what();
       std::cout << "exception caught: " << err_msg << std::endl;
       exitCode = dTRUE;
   }

   return exitCode;
}


int CRAlgoCalcWorldEndEffectorVec(CRHandInfo 	handInfo,
                                  Mat& 			rvecsWorldEndEffector,
                                  Mat& 			tvecsWorldEndEffector)
{
    int exitCode = dFALSE;

    try
    {
        double matrix[6] = {0.0};

        matrix[0] = handInfo.x;
        matrix[1] = handInfo.y;
        matrix[2] = handInfo.z;
        matrix[3] = handInfo.rx;
        matrix[4] = handInfo.ry;
        matrix[5] = handInfo.rz;

        #ifdef dVAMODEL
        //cout << "get 6 axis's [R|T]:" << endl;
        #else
        cout << "get 4 axis's [R|T]:" << endl;

        double     r       = 95,  // radius 95 mm
                   zShift  = 0.0,
                   dx      = 0.0,
                   dy      = 0.0,
                   dz_X    = 0.0,
                   dz_Y    = 0.0;

        dz_X   = r - (r * sin(((matrix[3]+90)/180) * dPI));
        dz_Y   = r - (r * sin(((matrix[4]+90)/180) * dPI));

        dy     = fabs(r * cos(((matrix[3]+90)/180) * dPI));
        dx     = fabs(r * cos(((matrix[4]+90)/180) * dPI));

        zShift = dz_X + dz_Y;

        matrix[0] -= dy;
        matrix[1] += dx;
        matrix[2] -= zShift;
        #endif

        tvecsWorldEndEffector.at<double>(0, 0) = matrix[0];
        tvecsWorldEndEffector.at<double>(1, 0) = matrix[1];
        tvecsWorldEndEffector.at<double>(2, 0) = matrix[2];
        rvecsWorldEndEffector.at<double>(0, 0) = matrix[3];
        rvecsWorldEndEffector.at<double>(1, 0) = matrix[4];
        rvecsWorldEndEffector.at<double>(2, 0) = matrix[5];

    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcWorldEndEffectorRmat(Mat 	rvecsWorldEndEffector,
                                   Mat&	rmatWorldEndEffector)
{
    int exitCode = dFALSE;

    try
    {
        CRAngleInfo angle;

        angle.rx = rvecsWorldEndEffector.at<double>(0, 0);
        angle.ry = rvecsWorldEndEffector.at<double>(1, 0);
        angle.rz = rvecsWorldEndEffector.at<double>(2, 0);

        exitCode = CRAlgoCalcAngleToRmat(angle,
                                         rmatWorldEndEffector);
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcEndEffectorCamRmat(Mat 	rvecsWorldEndEffector,
                                 Mat&	rmatEndEffectorCam)
{
    int exitCode = dFALSE;

    CRAngleInfo angle;

    try
    {
        angle.rx = rvecsWorldEndEffector.at<double>(0, 0);
        angle.ry = rvecsWorldEndEffector.at<double>(1, 0);
        angle.rz = rvecsWorldEndEffector.at<double>(2, 0);

        CRAlgoCalcAngleToRmat(angle,
                              rmatEndEffectorCam);

        rmatEndEffectorCam = rmatEndEffectorCam * rvecsWorldEndEffector;
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcCamObjectMat( Mat 	rmatCamObject,
                            Mat 	tvecsCamObject,
                            Mat&	rmatRefineCamObject,
                            Mat&	tvecsRefineCamObject)
{
    int exitCode = dFALSE;

    try
    {

        CRAngleInfo angle;

        /*--------------------------
         * rotate 0 degree
         */

        angle.rx = 0.0;
        angle.ry = 0.0;
        angle.rz = 0.0;

        CRAlgoCalcAngleToRmat(angle,
                              rmatRefineCamObject);

        tvecsCamObject.copyTo(tvecsRefineCamObject);

        rmatRefineCamObject =  rmatRefineCamObject * rmatCamObject;
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoCalcProjectionMat(Mat 	rmatCamObject,
                            Mat 	tvecsCamObject,
                            Mat 	cameraMatrix,
                            Mat&	invHMat)
{

    int exitCode = dFALSE;

    try
    {
        Mat extrinsicMatrix,
            projectionMatrix,
            homographyMatrix,
            invhomoM(3, 3, CV_64F);

        hconcat(rmatCamObject, tvecsCamObject, extrinsicMatrix);

        projectionMatrix = cameraMatrix * extrinsicMatrix;

        double p11 = projectionMatrix.at<double>(0, 0),
               p12 = projectionMatrix.at<double>(0, 1),
               p14 = projectionMatrix.at<double>(0, 3),
               p21 = projectionMatrix.at<double>(1, 0),
               p22 = projectionMatrix.at<double>(1, 1),
               p24 = projectionMatrix.at<double>(1, 3),
               p31 = projectionMatrix.at<double>(2, 0),
               p32 = projectionMatrix.at<double>(2, 1),
               p34 = projectionMatrix.at<double>(2, 3);

        homographyMatrix = (Mat_<double>(3, 3) << p11, p12, p14, p21, p22, p24, p31, p32, p34);

        invhomoM = homographyMatrix.inv();

        invhomoM.copyTo(invHMat);

        extrinsicMatrix.release();
        projectionMatrix.release();
        homographyMatrix.release();
        invhomoM.release();
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcWorldPoints(std::vector<Point2f>        imagePoints,
                           Mat                        oMiMatrix,
                           Mat                        cMoRmatrix,
                           Mat                        eMcRmatrix,
                           Mat                        wMeShotRmatrix,
                           Mat                        wMeTouchRmatrix,
                           Mat                        tvecsCamObject,
                           Mat                        tvecsEndEffectorCam,
                           Mat                        tvecsShotWorldEndEffector,
                           Mat                        tvecsTouchWorldEndEffector,
                           Mat                        tvecsEndEffectorTool,
                           std::vector<Point3f>*      worldPoints,
                           std::vector<CRAngleInfo>*  worldAngleList)

{
    int exitCode = dFALSE,
        i        = 0;

    try
    {
        #ifdef CV_ROTATION_DEBUG
        tvecsEndEffectorCam.at<double>(0,0) = 0;
        tvecsEndEffectorCam.at<double>(1,0) = 0;
        tvecsEndEffectorCam.at<double>(2,0) = 0;

        tvecsShotWorldEndEffector.at<double>(0,0) = 0;
        tvecsShotWorldEndEffector.at<double>(1,0) = 0;
        tvecsShotWorldEndEffector.at<double>(2,0) = 0;

        tvecsCamObject.at<double>(0,0) = 0;
        tvecsCamObject.at<double>(1,0) = 0;
        tvecsCamObject.at<double>(2,0) = 0;
        #endif

        // combine rotation matrix and translate vec (homogeneous matrix)
        Mat cMo(4, 4, CV_64F),
            eMc(4, 4, CV_64F),
            wMe(4, 4, CV_64F),
            wMo(4, 4, CV_64F);

        Mat toolShift(3, 1, CV_64F);

        CRAngleInfo resultAngle;

        exitCode = CRAlgoCalcHomogeneousMat(tvecsCamObject,
                                          cMoRmatrix,
                                          cMo);
        if(exitCode)
        {
            return exitCode;
        }

        exitCode = CRAlgoCalcHomogeneousMat(tvecsEndEffectorCam,
                                            eMcRmatrix,
                                            eMc);

        if(exitCode)
        {
            return exitCode;
        }

        exitCode = CRAlgoCalcHomogeneousMat(tvecsShotWorldEndEffector,
                                            wMeShotRmatrix,
                                            wMe);

        if(exitCode)
        {
            return exitCode;
        }

        wMo = wMe * eMc * cMo;

        toolShift = (wMeTouchRmatrix * tvecsEndEffectorTool);

        CRAlgoCalcRmatToAngle(wMeTouchRmatrix,
                              resultAngle);

        // alex_debug 0730
        #if 0
        CRAngleInfo oMcAngle,
                    cMeAngle,
                    eMwAngle,
                    cMwAngle;

        Mat         oMcRmatrix,
                    cMeRmatrix,
                    eMwRmatrix,
                    cMwRmatrix;

        oMcRmatrix = cMoRmatrix.inv();

        cMeRmatrix = eMcRmatrix.inv();

        eMwRmatrix = wMeShotRmatrix.inv();

        cMwRmatrix = cMeRmatrix * eMwRmatrix;

        CRAlgoCalcRmatToAngle(oMcRmatrix,
                              oMcAngle);

        CRAlgoCalcRmatToAngle(cMeRmatrix,
                              cMeAngle);

        CRAlgoCalcRmatToAngle(eMwRmatrix,
                              eMwAngle);

        CRAlgoCalcRmatToAngle(cMwRmatrix,
                              cMwAngle);

        cout << "translate: cMo: " << tvecsCamObject.t() << endl
             << "translate: eMc: " << tvecsEndEffectorCam.t() << endl
             << "translate: wMe: " << tvecsShotWorldEndEffector.t() << endl
             << "tool shift: " << toolShift.t() << endl;

        cout << "Angle: oMc: " << oMcAngle.rx << " " << oMcAngle.ry << " " << oMcAngle.rz << endl
             << "Angle: cMe: " << cMeAngle.rx << " " << cMeAngle.ry << " " << cMeAngle.rz << endl
             << "Angle: eMw: " << eMwAngle.rx << " " << eMwAngle.ry << " " << eMwAngle.rz << endl
             << "Angle: cMw: " << cMwAngle.rx << " " << cMwAngle.ry << " " << cMwAngle.rz << endl;
        #endif

        for(i = 0; i < (int)imagePoints.size(); i++)
        {
            Point3f resultPoint;

            Mat point2D = (Mat_<double>(3, 1) << imagePoints[i].x, imagePoints[i].y, 1);

            Mat point3Dw = oMiMatrix * point2D;

            double w = point3Dw.at<double>(2, 0);

            // P' = RP + T
            Mat objectPoint(4, 1, CV_64F),
                worldPoint(4, 1, CV_64F),
                worldWithToolPoint(3, 1, CV_64F);

            objectPoint.at<double>(0,0) = (double)point3Dw.at<double>(0,0)/w;
            objectPoint.at<double>(1,0) = (double)point3Dw.at<double>(1,0)/w;
            objectPoint.at<double>(2,0) = 0.0;
            objectPoint.at<double>(3,0) = 1;

            worldPoint = wMo * objectPoint;

            worldWithToolPoint.at<double>(0,0) = (double)worldPoint.at<double>(0,0);
            worldWithToolPoint.at<double>(1,0) = (double)worldPoint.at<double>(1,0);
            worldWithToolPoint.at<double>(2,0) = (double)worldPoint.at<double>(2,0);

            worldWithToolPoint += toolShift;

            // alex_debug 0925
            #if 0
            if(i == 0)
            {
                Mat cameraPoint(4, 1, CV_64F);

                cameraPoint = cMo * objectPoint;

                cout << "-----------------" << endl;
                cout << "image P to object P:" << endl << imagePoints[i] << "->" << objectPoint.t() << endl;
                cout << "object P to camera P:" << endl << objectPoint.t() << "->" << cameraPoint << endl;

                cout << "object P to world P:" << endl << objectPoint.t() << "->" << worldPoint.t() << endl;
                cout << "world P to world with tool P:" << endl << worldPoint.t() << "->" << worldWithToolPoint.t() << endl;
            }
            #endif

            resultPoint.x = worldWithToolPoint.at<double>(0,0);
            resultPoint.y = worldWithToolPoint.at<double>(1,0);
            resultPoint.z = tvecsTouchWorldEndEffector.at<double>(2,0);

            worldPoints->push_back(resultPoint);
            worldAngleList->push_back(resultAngle);
        }

        //release
        cMo.release();
        eMc.release();
        wMe.release();
        wMo.release();
        toolShift.release();
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoGetCaliImgInfo(unsigned char*                       rawData,
                         int                                  width,
                         int                                  height,
                         std::string                          imgInfoStr,
                         std::vector<std::vector<Point2f>>*		imgFeature,
                         bool*                                foundFlag)
{
    // The exit code of the sample application.
    int exitCode = dFALSE;

    bool found = false;

    Size grids(dGRID_WIDTH_OF_NUM,dGRID_HEIGHT_OF_NUM); //number of centers

    try
    {
        Mat srcImg(Size(width, height), CV_8UC1, rawData, Mat::AUTO_STEP);

        std::vector<Point2f> corners,
                             sortingCorners;

        found = findChessboardCorners(srcImg, grids, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (found)
        {
            Mat     color;
            std::vector<int> compression_params;
            compression_params.push_back(IMWRITE_PNG_COMPRESSION);

            cornerSubPix(srcImg, corners, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.1));//0.1 is accuracy

            sortingCorners = CRAlgoSortCornerPoints(corners);

            imgFeature->push_back(sortingCorners);

            cvtColor(srcImg, color, COLOR_GRAY2RGB);

            drawChessboardCorners(color, grids, Mat(corners), found);

            imwrite(imgInfoStr, color, compression_params);

            color.release();
            std::vector<Point2f>().swap(sortingCorners);
            std::vector<int>().swap(compression_params);
        }

        *foundFlag = found;
         srcImg.release();
         std::vector<Point2f>().swap(corners);
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoGetCaliHandInfo(double                		x,
                          double                		y,
                          double                		z,
                          double                		rx,
                          double                		ry,
                          double                		rz,
                          std::vector<CRHandInfo>*		handInfo)
{
    // The exit code of the sample application.
    int exitCode = dFALSE;

    CRHandInfo info;

    info.x = x;
    info.y = y;
    info.z = z;
    info.rx = rx;
    info.ry = ry;
    info.rz = rz;

    handInfo->push_back(info);

    return exitCode;
}

int CRAlgoCalcCameraMat(int                               gridSize,
                        std::vector<std::vector<Point2f>> pointList,
                        Size                              imageSize,
                        Mat&                              cameraMatrix,
                        Mat&                              distCoeffs,
                        std::vector<Mat>*                 rmatCamObjectList,
                        std::vector<Mat>*                 tvecsCamObjectList)
{
    // The exit code of the sample application.
    int exitCode = dFALSE;

    double  rms    = 0.0;

    //number of centers
    Size grids(dGRID_WIDTH_OF_NUM,dGRID_HEIGHT_OF_NUM);

    try
    {
        std::vector<std::vector<Point3f>> objectList;

        std::vector<Mat> rvecsCamList;

        objectList = CRAlgoCalcBoardCornerPositionsList(grids.width, grids.height, gridSize, pointList.size());

        rms = calibrateCamera(objectList, pointList, imageSize, cameraMatrix, distCoeffs, rvecsCamList, *tvecsCamObjectList);

        for(int i = 0; i < (int)rvecsCamList.size(); i++)
        {
            Mat R(3, 3, CV_64FC1);

            Rodrigues(rvecsCamList[i], R);

            rmatCamObjectList->push_back(R);

            R.release();
        }

        std::vector<std::vector<Point3f>>().swap(objectList);
        std::vector<Mat>().swap(rvecsCamList);

        if(rms > 1.0)
        {
          std::cout << "rms is too large:" << rms << std::endl;
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcEndEffectorToolVec(double     	toolWidth,
                                 double      	toolHeight,
                                 double      	toolTheta,
                                 Mat&			tvecsEndEffectorTool)
{
    // The exit code of the sample application.
    int exitCode = dFALSE;

    try
    {
        double toolTx = (toolWidth * cos((toolTheta/180) * dPI));
        double toolTy = (toolWidth * sin((toolTheta/180) * dPI));

        tvecsEndEffectorTool.at<double>(0,0) = -toolTx;
        tvecsEndEffectorTool.at<double>(1,0) = -toolTy;
        tvecsEndEffectorTool.at<double>(2,0) = -toolHeight;
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}

int CRAlgoCalcWorldEndEffectorMat(std::vector <CRHandInfo> 	handInfoList,
                                  std::vector<Mat>*         tvecsWorldEndEffectorList,
                                  std::vector<Mat>*         rmatWorldEndEffectorList)
{
    int exitCode = dFALSE,
        i        = 0;

    try
    {
        for(i = 0; i < (int)handInfoList.size(); i++)
        {
            CRAngleInfo angle;

            Mat rmatWorldEndEffector(3, 3, CV_64F),
                tvecsWorldEndEffector(3, 1, CV_64F);

            tvecsWorldEndEffector.at<double>(0, 0) = handInfoList[i].x;
            tvecsWorldEndEffector.at<double>(1, 0) = handInfoList[i].y;
            tvecsWorldEndEffector.at<double>(2, 0) = handInfoList[i].z;

            angle.rx = handInfoList[i].rx;
            angle.ry = handInfoList[i].ry;
            angle.rz = handInfoList[i].rz;

            CRAlgoCalcAngleToRmat(angle,
                                  rmatWorldEndEffector);

            rmatWorldEndEffectorList->push_back(rmatWorldEndEffector);

            tvecsWorldEndEffectorList->push_back(tvecsWorldEndEffector);

            rmatWorldEndEffector.release();
            tvecsWorldEndEffector.release();
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    return exitCode;
}


int CRAlgoCalcEndEffectorCamMat(std::vector<Mat> 	tvecsWorldEndEffectorList,
                                std::vector<Mat> 	rmatWorldEndEffectorList,
                                std::vector<Mat> 	tvecsCamObjectList,
                                std::vector<Mat> 	rmatCamObjectList,
                                Mat&				tvecsEndEffectorCam,
                                Mat&				rmatEndEffectorCam)
{
    int exitCode = dFALSE,
        i        = 0;

    #ifdef CV_CALIBRATION_TSAI
    CV_Assert(tvecsWorldEndEffectorList.size() == tvecsCamObjectList.size());
    CV_Assert(rmatWorldEndEffectorList.size() == rmatCamObjectList.size());

    try
    {
        std::vector<Mat>  wMeList,
                          cMoList,
                          refeMeList,
                          refcMcList;

        Mat eMc(4, 4, CV_64FC1);

        for(i = 0;  i < (int)rmatCamObjectList.size(); i++)
        {
            Mat wMe(4, 4, CV_64FC1),
                cMo(4, 4, CV_64FC1);

            exitCode = CRAlgoCalcHomogeneousMat(tvecsWorldEndEffectorList[i],
                                                rmatWorldEndEffectorList[i],
                                                wMe);

            if(exitCode)
            {
                return dTRUE;
            }

            exitCode = CRAlgoCalcHomogeneousMat(tvecsCamObjectList[i],
                                                rmatCamObjectList[i],
                                                cMo);

            if(exitCode)
            {
                return dTRUE;
            }

            wMeList.push_back(wMe);
            cMoList.push_back(cMo);

            wMe.release();
            cMo.release();
        }

        for(i = 0;  i < (int)wMeList.size() - 1; i++)
        {
            Mat refeMe(4, 4, CV_64FC1),
                refcMc(4, 4, CV_64FC1);

            wMeList[i].copyTo(refeMe);
            cMoList[i].copyTo(refcMc);

            refeMe = refeMe.inv() * wMeList[i + 1];
            refcMc = refcMc * cMoList[i + 1].inv();

            // alex_debug 0813
            #if 0
            Mat rmatRefEndEnd(3, 3, CV_64FC1),
                tvecsRefEndEnd(3, 1, CV_64FC1);

            refeMe(Rect(0, 0, 3, 3)).copyTo(rmatRefEndEnd);
            refeMe(Rect(3, 0, 1, 3)).copyTo(tvecsRefEndEnd);
            CRAngleInfo angleRefEndEnd;

            CRAlgoCalcRmatToAngle(rmatRefEndEnd,
                                  angleRefEndEnd);

            cout << "RefEndEnd_" << i << " angle: "
                 << angleRefEndEnd.rx  << " "
                 << angleRefEndEnd.ry  << " "
                 << angleRefEndEnd.rz  << " "
                 << "tvecsRefEndEnd:" << tvecsRefEndEnd.t() << endl;
            #endif

            #if 0
            Mat rmatRefCamCam(3, 3, CV_64FC1),
                tvecsRefCamCam(3, 1, CV_64FC1);

            refcMc(Rect(0, 0, 3, 3)).copyTo(rmatRefCamCam);
            refcMc(Rect(3, 0, 1, 3)).copyTo(tvecsRefCamCam);
            CRAngleInfo angleRefCamCam;

            CRAlgoCalcRmatToAngle(rmatRefCamCam,
                                  angleRefCamCam);

            cout << "RefCamCam_" << i << " angle: "
                 << angleRefCamCam.rx  << " "
                 << angleRefCamCam.ry  << " "
                 << angleRefCamCam.rz  << " "
                 << "tvecsRefCamCam:" << tvecsRefCamCam.t() << endl;
            #endif

            refeMeList.push_back(refeMe);

            refcMcList.push_back(refcMc);

            refeMe.release();
            refcMc.release();
        }

        CRTsaiHandEye(eMc,
                      refeMeList,
                      refcMcList);
        #if 0
        // alex_debug 0815
        Mat AXResult(4, 4, CV_64FC1),
            XBResult(4, 4, CV_64FC1),
            diffResult(4, 4, CV_64FC1);

        AXResult = refeMeList[0] * eMc;
        XBResult = eMc * refcMcList[0];
        diffResult = AXResult - XBResult;

        cout << "diffResult:" << endl << diffResult << endl;
        #endif

        eMc(Rect(0, 0, 3, 3)).copyTo(rmatEndEffectorCam);
        eMc(Rect(3, 0, 1, 3)).copyTo(tvecsEndEffectorCam);

        std::vector<Mat>().swap(wMeList);
        std::vector<Mat>().swap(cMoList);
        std::vector<Mat>().swap(refeMeList);
        std::vector<Mat>().swap(refcMcList);

        eMc.release();

        // alex_debug 0813
        #if 0
        CRAngleInfo angleEC,
                    angleCE;

        Mat rmatCamEndEffector(3, 3, CV_64FC1),
            tvecsCamEndEffector(3, 1, CV_64FC1);

        cMe = eMc.inv();

        cMe(Rect(0, 0, 3, 3)).copyTo(rmatCamEndEffector);
        cMe(Rect(3, 0, 1, 3)).copyTo(tvecsCamEndEffector);

        CRAlgoCalcRmatToAngle(rmatEndEffectorCam,
                              angleEC);

        CRAlgoCalcRmatToAngle(rmatCamEndEffector,
                              angleCE);

        cout << "EndEffectorCamAngle: " << angleEC.rx << " " << angleEC.ry << " " << angleEC.rz << endl;
        cout << "tvecsEndEffectorCam: " << tvecsEndEffectorCam.t() << endl;

        cout << "CamEndEffectorAngle: " << angleCE.rx << " " << angleCE.ry << " " << angleCE.rz << endl;
        cout << "tvecsCamEndEffector: " << tvecsCamEndEffector.t() << endl;
        #endif
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
        exitCode = dTRUE;
    }

    #else
    CRAngleInfo angle;

    // Rotation matrix is assumed to be the same as world(TODO)
    #if 0
    rmatEndEffectorCam.at<double>(0, 0) = 1;
    rmatEndEffectorCam.at<double>(0, 1) = 0;
    rmatEndEffectorCam.at<double>(0, 2) = 0;
    rmatEndEffectorCam.at<double>(1, 0) = 0;
    rmatEndEffectorCam.at<double>(1, 1) = 1;
    rmatEndEffectorCam.at<double>(1, 2) = 0;
    rmatEndEffectorCam.at<double>(2, 0) = 0;
    rmatEndEffectorCam.at<double>(2, 1) = 0;
    rmatEndEffectorCam.at<double>(2, 2) = 1;

    #else

    angle.rx = 0;
    angle.ry = -60;
    angle.rz = -135;

    CRAlgoCalcAngleToRmat(angle,
                          rmatEndEffectorCam);
    #endif


    //end effector frame's camera vec

    double cameraTx = (cameraWidth * cos((cameraTheta/180) * dPI));
    double cameraTy = (cameraWidth * sin((cameraTheta/180) * dPI));

    tvecsEndEffectorCam.at<double>(0, 0) = cameraTx;
    tvecsEndEffectorCam.at<double>(1, 0) = cameraTy;
    tvecsEndEffectorCam.at<double>(2, 0) = cameraHeight;

    #if 1
    // alex_debug 0801
    cout << "hand to eye rotate angle: " << angle.rx << " "
         << angle.ry << " "
         << angle.rz << endl;

    cout << "hand to eye vec(eVec): " << tvecsEndEffectorCam.t() << endl;
    #endif

    #endif

    return exitCode;
}


