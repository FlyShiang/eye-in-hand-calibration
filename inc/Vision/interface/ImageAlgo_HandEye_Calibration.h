/*
 * Author: Yu-Shiang Yan
 *
 * Note: Eye in hand calibration interface(Ex. camera calibration, hand eye calibration)
 */

#ifndef IMAGEALGO_HANDEYE_CALIBRATION_H
#define IMAGEALGO_HANDEYE_CALIBRATION_H

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#ifdef __cplusplus
extern "C" {
#endif

#define dPI 3.14159265358979323846
#define dGRID_WIDTH_OF_NUM 13
#define dGRID_HEIGHT_OF_NUM 9


class CRAngleInfo {
public:
    double rx;
    double ry;
    double rz;
};

class CRHandInfo {
public:
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};

int CRAlgoGetCaliImgInfo(unsigned char*                         rawData,
                         int                     		width,
                         int                     		height,
                         std::string                		imgInfoStr,
                         std::vector<std::vector<Point2f>>*	imgFeature,
                         bool*                                  foundFlag);


int CRAlgoGetCaliHandInfo(double                		x,
                          double                		y,
                          double                		z,
                          double                		rx,
                          double                		ry,
                          double                		rz,
                          std::vector<CRHandInfo>*		handInfo);


int CRAlgoCalcCameraMat(int                     		gridSize,
                        std::vector<std::vector<Point2f> > 	pointList,
                        Size                    		imageSize,
                        Mat&					cameraMatrix,
                        Mat&					distCoeffs,
                        std::vector<Mat>*                       rmatCamObjectList,
                        std::vector<Mat>*			tvecsCamList);


int CRAlgoCalcWorldEndEffectorMat(std::vector <CRHandInfo> 			handInfoList,
                                  std::vector<Mat>* 				tvecsWorldEndEffectorList,
                                  std::vector<Mat>* 				rmatWorldEndEffectorList);

int CRAlgoCalcEndEffectorCamMat(std::vector<Mat> 	tvecsWorldEndEffectorList,
                                std::vector<Mat> 	rmatWorldEndEffectorList,
                                std::vector<Mat> 	tvecsCamObjectList,
                                std::vector<Mat> 	rmatCamObjectList,
                                Mat&			tvecsEndEffectorCam,
                                Mat&			rmatEndEffectorCam);


int CRAlgoCalcCamObjectMat( Mat 	rmatCamObject,
                            Mat 	tvecsCamObject,
                            Mat&	rmatRefineCamObject,
                            Mat&	tvecsRefineCamObject);


int CRAlgoCalcProjectionMat(Mat 	rmatCamObject,
                            Mat 	tvecsCamObject,
                            Mat 	cameraMatrix,
                            Mat&	invHMat);


int CRAlgoCalcWorldEndEffectorVec(CRHandInfo 	handInfo,
                                  Mat&          rvecsWorldEndEffector,
                                  Mat&          tvecsWorldEndEffector);

int CRAlgoCalcEndEffectorToolVec(double      	toolWidth,
                                 double      	toolHeight,
                                 double      	toolTheta,
                                 Mat&			tvecsEndEffectorTool);

int CRAlgoCalcWorldEndEffectorRmat(Mat 		rvecsWorldEndEffector,
                                   Mat&		rmatWorldEndEffector);

int CRAlgoCalcEndEffectorCamRmat(Mat 	rvecsWorldEndEffector,
                                 Mat&	rmatEndEffectorCam);

int CRAlgoTestChessBoard(Mat             		srcImg,
                         std::vector<Point2f>*          ImageCorners);


int CRAlgoCalcWorldPoints(std::vector<Point2f>                          imagePoints,
                           Mat 						oMiMatrix,
                           Mat 						cMoRmatrix,
                           Mat 						eMcRmatrix,
                           Mat 						wMeShotRmatrix,
                           Mat		 				wMeTouchRmatrix,
                           Mat 						tvecsCamObject,
                           Mat 						tvecsEndEffectorCam,
                           Mat 						tvecsShotWorldEndEffector,
                           Mat                                          tvecsTouchWorldEndEffector,
                           Mat 						tvecsEndEffectorTool,
                           std::vector<Point3f>*                        worldPoints,
                           std::vector<CRAngleInfo>*                    worldAngleList);


int CRAlgoCalcRmatToAngle(Mat 		R,
                          CRAngleInfo& 	angle);

int CRAlgoCalcAngleToRmat(CRAngleInfo angle,
                          Mat&        R);

int CRAlgoCalcHomogeneousMat(Mat     translateVec,
                             Mat     rotationMat,
                             Mat&    homogeneousMat);


#ifdef __cplusplus
}
#endif

#endif // IMAGEALGO_HANDEYE_CALIBRATION_H

