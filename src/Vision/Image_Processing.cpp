/*
 * Author: Yu-Shiang Yan
 *
 * Note: Image processing interface
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

#include <ImageAlgo_HandEye_Calibration.h>
#include <Image_Processing.h>
#include <Image_Params.h>
#include <Utility.h>

int CR_SaveAsImg(CR_Image* image,
                  char*	   img_str)
{
    int exitCode = dFALSE;

    if((image == 0) || (img_str == 0))
    {
        return eIMAGE_PROCESS_SAVE_IMG_SRC_INPUT;
    }

    exitCode = CRAlgoCVSaveAsImg(image->raw_data,
                                 image->width,
                                 image->height,
                                 image->pitch,
                                 img_str);

    if(exitCode)
    {
        return eIMAGE_PROCESS_SAVE_IMG;
    }
	
    return exitCode;
}


void CRHandEyeParamsClear(CR_EyeInHand_Cali_Param*        pCaliInfo)
{
    int i;

    if(pCaliInfo)
    {
        pCaliInfo->calibrationHandInfo.clear();

        for(i = 0; i < (int)pCaliInfo->calibrationImgFeature.size(); i++)
        {
            pCaliInfo->calibrationImgFeature[i].clear();
        }

        pCaliInfo->calibrationImgFeature.clear();

    }
}


int CR_Image_Resource_Init(CR_Image** image)
{
    int	exitCode = dFALSE;

    (*image)  = (CR_Image*)malloc(sizeof(CR_Image));

    if((*image))
    {
        memset((*image), 0, sizeof(CR_Image));

        (*image)->raw_data = (unsigned char*)malloc((CR_MAX_WIDTH * CR_MAX_HEIGHT * CR_MAX_PITCH_SIZE));

        if((*image)->raw_data == NULL)
        {
            return eIMAGE_PROCESS_IMG_INIT;
        }

        memset((*image)->raw_data, 0, (CR_MAX_WIDTH * CR_MAX_HEIGHT * CR_MAX_PITCH_SIZE) );

        (*image)->mask_data = (unsigned char*)malloc(CR_MAX_WIDTH * CR_MAX_HEIGHT);

        if((*image)->mask_data == NULL)
        {
            return eIMAGE_PROCESS_IMG_INIT;
        }

        memset((*image)->mask_data, 255, CR_MAX_WIDTH * CR_MAX_HEIGHT);
    }
    else
    {
        return eIMAGE_PROCESS_IMG_INIT;
    }

    return exitCode;
}

void CR_Image_Resource_Release(CR_Image* image)
{
    if(image)
    {
        if(image->raw_data)
        {
            free(image->raw_data);
        }

        if(image->mask_data)
        {
            free(image->mask_data);
        }

        free(image);
    }
}

void CRHandEyeResultClear(CR_EyeInHand_Result *result)
{
    if(result)
    {
        result->imgPList.clear();
        result->worldPList.clear();
        result->worldAngleList.clear();
    }
}


void CRHandEyeResultRelease(CR_EyeInHand_Result *result)
{

    std::vector<Point2f>().swap(result->imgPList);
    std::vector<Point3f>().swap(result->worldPList);
    std::vector<CRAngleInfo>().swap(result->worldAngleList);

    delete result;
}


int CRHandEyeResultInit(CR_EyeInHand_Result **result)
{
    int exitCode     = dFALSE;

    (*result)  = new CR_EyeInHand_Result;

    if((*result))
    {

    }
    else
    {
        exitCode = eIMAGE_PROCESS_HAND_EYE_CALIBRATION_RESULT_INIT;
    }

    return exitCode;
}


void CRHandEyeCaliParamRelease(CR_EyeInHand_Cali_Param *caliParam)
{
    caliParam->cMoRmatrix.release();
    caliParam->oMiMatrix.release();
    caliParam->eMcRmatrix.release();
    caliParam->wMeShotRmatrix.release();
    caliParam->wMeTouchRmatrix.release();
    caliParam->tvecsCamObject.release();
    caliParam->tvecsEndEffectorCam.release();
    caliParam->tvecsShotWorldEndEffector.release();
    caliParam->tvecsTouchWorldEndEffector.release();
    caliParam->tvecsEndEffectorTool.release();

    delete caliParam;
}

int CRHandEyeCaliParamInit(CR_EyeInHand_Cali_Param **caliParam)
{
    int exitCode     = dFALSE;

    (*caliParam)  = new CR_EyeInHand_Cali_Param;

    if((*caliParam))
    {
        Mat cMoRmatrix(3, 3, CV_64F),
            oMiMatrix(3, 3, CV_64F),
            eMcRmatrix(3, 3, CV_64F),
            wMeShotRmatrix(3, 3, CV_64F),
            wMeTouchRmatrix(3, 3, CV_64F),
            tvecsCamObject(3, 1, CV_64F),
            tvecsEndEffectorCam(3, 1, CV_64F),
            tvecsShotWorldEndEffector(3, 1, CV_64F),
            tvecsTouchWorldEndEffector(3, 1, CV_64F),
            tvecsEndEffectorTool(3, 1, CV_64F);

        (*caliParam)->cMoRmatrix = cMoRmatrix;
        (*caliParam)->oMiMatrix = oMiMatrix;
        (*caliParam)->eMcRmatrix = eMcRmatrix;
        (*caliParam)->wMeShotRmatrix = wMeShotRmatrix;
        (*caliParam)->wMeTouchRmatrix = wMeTouchRmatrix;
        (*caliParam)->tvecsCamObject = tvecsCamObject;
        (*caliParam)->tvecsEndEffectorCam = tvecsEndEffectorCam;
        (*caliParam)->tvecsShotWorldEndEffector = tvecsShotWorldEndEffector;
        (*caliParam)->tvecsTouchWorldEndEffector =tvecsTouchWorldEndEffector;
        (*caliParam)->tvecsEndEffectorTool = tvecsEndEffectorTool;
    }
    else
    {
        exitCode = eIMAGE_PROCESS_HAND_EYE_CALIBRATION_PARAM_INIT;
    }

    return exitCode;
}


int CRHandEyeGetCalibrationInfo(unsigned char* rawData,
                                int            width,
                                int            height,
                                double         posX,
                                double         posY,
                                double         posZ,
                                double         posRx,
                                double         posRy,
                                double         posRz,
                                std::string         imgInfoStr,
                                CR_EyeInHand_Cali_Param  *pCaliInfo,
                                bool           *found)
{
    int exitCode = dFALSE;

    bool foundCorner = false;

    if(pCaliInfo == 0)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_SRC_INPUT;
    }

    pCaliInfo->imgResolution = Size(width,height);

    exitCode = CRAlgoGetCaliImgInfo(rawData,
                                    width,
                                    height,
                                    imgInfoStr,
                                    &pCaliInfo->calibrationImgFeature,
                                    &foundCorner);
    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_GET_IMG_INFO;
    }

    if(foundCorner)
    {
        exitCode = CRAlgoGetCaliHandInfo(posX,
                                         posY,
                                         posZ,
                                         posRx,
                                         posRy,
                                         posRz,
                                         &pCaliInfo->calibrationHandInfo);
        if(exitCode)
        {
            return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_GET_HAND_INFO;
        }
    }

    *found = foundCorner;

    return exitCode;
}


int CRHandEyeCalcCalibration(CR_EyeInHand_Cali_Param*  pCaliInfo)
{
    int exitCode      = dFALSE;

    std::vector<Mat> rmatCamObjectList,
                     tvecsCamObjectList,
                     rmatWorldEndEffectorList,
                     tvecsWorldEndEffectorList;

    Mat cameraMatrix(3, 3, CV_64F),
        distCoeffs(1, 5, CV_64F);

    // calculate intrinsic matrix and extrinsic matrix
    exitCode = CRAlgoCalcCameraMat(pCaliInfo->gridSize,
                                   pCaliInfo->calibrationImgFeature,
                                   pCaliInfo->imgResolution,
                                   cameraMatrix,
                                   distCoeffs,
                                   &rmatCamObjectList,
                                   &tvecsCamObjectList);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_CAMERA_MATRIX;
    }

    // calculate camera to object matrix
    exitCode = CRAlgoCalcCamObjectMat(rmatCamObjectList[0],
                                      tvecsCamObjectList[0],
                                      pCaliInfo->cMoRmatrix,
                                      pCaliInfo->tvecsCamObject);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_EYE_OBJECT_MATRIX;
    }

    // calculate world to end effector matrix
    exitCode = CRAlgoCalcWorldEndEffectorMat(pCaliInfo->calibrationHandInfo,
                                             &tvecsWorldEndEffectorList,
                                             &rmatWorldEndEffectorList);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_WORLD_HAND_MATRIX;
    }

    // calculate end effector to camera matrix
    exitCode = CRAlgoCalcEndEffectorCamMat(tvecsWorldEndEffectorList,
                                           rmatWorldEndEffectorList,
                                           tvecsCamObjectList,
                                           rmatCamObjectList,
                                           pCaliInfo->tvecsEndEffectorCam ,
                                           pCaliInfo->eMcRmatrix);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_HAND_EYE_MATRIX;
    }


    // calculate projection matrix
    exitCode = CRAlgoCalcProjectionMat(rmatCamObjectList[0],
                                       tvecsCamObjectList[0],
                                       cameraMatrix,
                                       pCaliInfo->oMiMatrix);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_PROJECTION_MATRIX;
    }

    // calculate tool vec
    exitCode = CRAlgoCalcEndEffectorToolVec(pCaliInfo->toolWidth,
                                            pCaliInfo->toolHeight,
                                            pCaliInfo->toolTheta,
                                            pCaliInfo->tvecsEndEffectorTool);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_TOOL_VEC;
    }

    cameraMatrix.release();
    distCoeffs.release();
    std::vector<Mat>().swap(rmatCamObjectList);
    std::vector<Mat>().swap(tvecsCamObjectList);
    std::vector<Mat>().swap(rmatWorldEndEffectorList);
    std::vector<Mat>().swap(tvecsWorldEndEffectorList);

    return exitCode;
}

int CRHandEyeGetToolInfo(double     toolWidth,
                         double     toolHeight,
                         double     toolTheta,
                         CR_EyeInHand_Cali_Param*  pCaliInfo)
{
    int exitCode     = dFALSE;

    if(pCaliInfo == 0)
    {
       return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_GET_TOOL_INFO;
    }

    pCaliInfo->toolHeight = toolHeight;

    pCaliInfo->toolWidth = toolWidth;

    pCaliInfo->toolTheta = toolTheta;

    return exitCode;
}

int CRHandEyeGetRunTimeInfo(double     shotX,
                            double     shotY,
                            double     shotZ,
                            double     shotRx,
                            double     shotRy,
                            double     shotRz,
                            double     touchX,
                            double     touchY,
                            double     touchZ,
                            double     touchRx,
                            double     touchRy,
                            double     touchRz,
                            int        gridSize,
                            CR_EyeInHand_Cali_Param*  pCaliInfo)
{
    int exitCode     = dFALSE;

    Mat  rvecsShotWorldEndEffector(3, 1, CV_64F),
         rvecsTouchWorldEndEffector(3, 1, CV_64F);

    pCaliInfo->shotRunTimeHandInfo.x = shotX;
    pCaliInfo->shotRunTimeHandInfo.y = shotY;
    pCaliInfo->shotRunTimeHandInfo.z = shotZ;
    pCaliInfo->shotRunTimeHandInfo.rx = shotRx;
    pCaliInfo->shotRunTimeHandInfo.ry = shotRy;
    pCaliInfo->shotRunTimeHandInfo.rz = shotRz;

    pCaliInfo->touchHandInfo.x = touchX;
    pCaliInfo->touchHandInfo.y = touchY;
    pCaliInfo->touchHandInfo.z = touchZ;
    pCaliInfo->touchHandInfo.rx = touchRx;
    pCaliInfo->touchHandInfo.ry = touchRy;
    pCaliInfo->touchHandInfo.rz = touchRz;
    pCaliInfo->gridSize = gridSize;


    exitCode = CRAlgoCalcWorldEndEffectorVec(pCaliInfo->shotRunTimeHandInfo,
                                             rvecsShotWorldEndEffector,
                                             pCaliInfo->tvecsShotWorldEndEffector);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_SHOT_HAND_VEC;
    }


    exitCode = CRAlgoCalcWorldEndEffectorRmat(rvecsShotWorldEndEffector,
                                              pCaliInfo->wMeShotRmatrix);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_SHOT_HAND_RMATRIX;
    }


    exitCode = CRAlgoCalcWorldEndEffectorVec(pCaliInfo->touchHandInfo,
                                             rvecsTouchWorldEndEffector,
                                             pCaliInfo->tvecsTouchWorldEndEffector);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_TOUCH_HAND_VEC;
    }


    exitCode = CRAlgoCalcWorldEndEffectorRmat(rvecsTouchWorldEndEffector,
                                              pCaliInfo->wMeTouchRmatrix);

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_TOUCH_HAND_RMATRIX;
    }

    rvecsShotWorldEndEffector.release();
    rvecsTouchWorldEndEffector.release();

    return exitCode;
}

int CRHandEyeCalcImgToWorld(CR_EyeInHand_Cali_Param*        pCaliInfo,
                            std::vector<Point2f>            imgPointList,
                            std::vector<Point3f>*           worldPointList,
                            std::vector<CRAngleInfo>*       worldAngleList)
{
    int exitCode     = dFALSE;

    exitCode = CRAlgoCalcWorldPoints(imgPointList,
                                     pCaliInfo->oMiMatrix,
                                     pCaliInfo->cMoRmatrix,
                                     pCaliInfo->eMcRmatrix,
                                     pCaliInfo->wMeShotRmatrix,
                                     pCaliInfo->wMeTouchRmatrix,
                                     pCaliInfo->tvecsCamObject,
                                     pCaliInfo->tvecsEndEffectorCam,
                                     pCaliInfo->tvecsShotWorldEndEffector,
                                     pCaliInfo->tvecsTouchWorldEndEffector,
                                     pCaliInfo->tvecsEndEffectorTool,
                                     worldPointList,
                                     worldAngleList);
    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_WORLD_POINTS;
    }

    return exitCode;
}


int CR_ReadImg(CR_Image*   srcImg,
               char*       str)
{
    int exitCode = dFALSE;

    if(srcImg == 0)
    {
        return eIMAGE_PROCESS_READ_IMG_SRC_INPUT;
    }

    exitCode = CRAlgoCVReadImg(srcImg->raw_data,
                               &srcImg->width,
                               &srcImg->height,
                               &srcImg->pitch,
                               str);

    if(exitCode)
    {
        return eIMAGE_PROCESS_READ_IMG;
    }

    return exitCode;
}
