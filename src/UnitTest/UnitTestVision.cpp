/*
 * Vision Module Unit Testing
 * 
 * Author: Yu-Shiang Yan
 * Overview: First we will get calibration information(Ex. hand coordinates, chessboard pattern). Last we will caculate camera
 * calibration and hnad-eye calibration.
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <Utility.h>
#include <Image_Processing.h>
#include <Image_Params.h>

#define dCHESS_BOARD_RUNTIME_IMG_NAME "1.bmp"

#define dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_PATH "../../../testImg/test1/"
#define dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_NUM 10
#define dHAND_EYE_CALIBRATION_HAND_INFO_NAME "calibration_hand_info.csv"
#define dHAND_EYE_CALIBRATION_GRID_SIZE 5

#define dHAND_EYE_CALIBRATION_TOOL_WIDTH 32.864
#define dHAND_EYE_CALIBRATION_TOOL_HEIGHT 143.351
#define dHAND_EYE_CALIBRATION_TOOL_THETA 138.365

static void handEyeCalibration_test(int argc, char* argv[])
{
    int     ret       = 0,
            i         = 0,
            ImgID     = 0;

    std::string      input_str,
                readImg_str,
                saveImg_str,
                calibrationName,
                testStr;

    CR_Image    *runtime_img    = 0;

    std::vector<CRHandInfo> handInfo;

    CR_EyeInHand_Cali_Param *caliParams = 0;

    CR_EyeInHand_Result *result = 0;

    testStr = dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_PATH;

    testStr += dCHESS_BOARD_RUNTIME_IMG_NAME;

    /* initialize image resource using Heap memory */
    ret = CR_Image_Resource_Init(&runtime_img);
    PRINT_ERROR_CODE(ret)

    /* initialize calibration resource using Heap memory */
    ret = CRHandEyeCaliParamInit(&caliParams);
    PRINT_ERROR_CODE(ret)

    /* initialize result resource using Heap memory */
    ret = CRHandEyeResultInit(&result);
    PRINT_ERROR_CODE(ret)


    calibrationName = dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_PATH;

    calibrationName += dHAND_EYE_CALIBRATION_HAND_INFO_NAME;

    ret = CRReadCalibrationHandInfo(&handInfo,
                                    calibrationName,
                                    dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_NUM);
    PRINT_ERROR_CODE(ret)

    printf("press 'n' or 'N' to grab calibration images/press 'e' or 'E' to leave\n");

    while(true)
    {
        std::getline(std::cin, input_str);

        if((input_str == "n" ) || (input_str == "N"))
        {
            bool found = false;

            readImg_str = dHAND_EYE_CALIBRATION_CRCALIBRATION_IMG_PATH;

            readImg_str += std::to_string(++ImgID) + ".bmp";

            saveImg_str = std::to_string(ImgID);

            saveImg_str += ".png";

            ret = CR_ReadImg(runtime_img,
                             (char*)readImg_str.c_str());
            PRINT_ERROR_CODE(ret)

            std::cout << readImg_str << std::endl;

            std::cout << saveImg_str << std::endl;

            ret = CRHandEyeGetCalibrationInfo(runtime_img->raw_data,
                                              runtime_img->width,
                                              runtime_img->height,
                                              handInfo[ImgID-1].x,
                                              handInfo[ImgID-1].y,
                                              handInfo[ImgID-1].z,
                                              handInfo[ImgID-1].rx,
                                              handInfo[ImgID-1].ry,
                                              handInfo[ImgID-1].rz,
                                              saveImg_str,
                                              caliParams,
                                              &found);
            PRINT_ERROR_CODE(ret)

            printf("found:%d\n",found);
        }

        if((input_str == "e") || (input_str == "E"))
        {
            break;
        }
    }

    for(i = 0; i < (int)caliParams->calibrationHandInfo.size(); i++)
    {
        std::cout << "handInfo: "
             << caliParams->calibrationHandInfo[i].x << " "
             << caliParams->calibrationHandInfo[i].y << " "
             << caliParams->calibrationHandInfo[i].z << " "
             << caliParams->calibrationHandInfo[i].rx << " "
             << caliParams->calibrationHandInfo[i].ry << " "
             << caliParams->calibrationHandInfo[i].rz << std::endl;

        std::cout << "featureInfo: "
             << caliParams->calibrationImgFeature[i][0].x << " "
             << caliParams->calibrationImgFeature[i][0].y << std::endl;
    }

    ret = CRHandEyeGetRunTimeInfo(caliParams->calibrationHandInfo[0].x,
                                  caliParams->calibrationHandInfo[0].y,
                                  caliParams->calibrationHandInfo[0].z,
                                  caliParams->calibrationHandInfo[0].rx,
                                  caliParams->calibrationHandInfo[0].ry,
                                  caliParams->calibrationHandInfo[0].rz,
                                  0,
                                  0,
                                  110,
                                  -180,
                                  0,
                                  0,
                                  dHAND_EYE_CALIBRATION_GRID_SIZE,
                                  caliParams);
    PRINT_ERROR_CODE(ret)

    ret = CRHandEyeGetToolInfo(dHAND_EYE_CALIBRATION_TOOL_WIDTH,
                               dHAND_EYE_CALIBRATION_TOOL_HEIGHT,
                               dHAND_EYE_CALIBRATION_TOOL_THETA,
                               caliParams);
    PRINT_ERROR_CODE(ret)


    ret = CRHandEyeCalcCalibration(caliParams);
    PRINT_ERROR_CODE(ret)


    ret = CRWriteCalibrationParams(caliParams,
                                   "caliParams.csv");
    PRINT_ERROR_CODE(ret)


    CRHandEyeParamsClear(caliParams);

    CRHandEyeResultClear(result);

    ret = CRReadCalibrationParams(caliParams,
                                  "caliParams.csv");
    PRINT_ERROR_CODE(ret)


    ret = CRChessBoardTesting(&result->imgPList,
                              testStr);
    PRINT_ERROR_CODE(ret)

    ret = CRHandEyeCalcImgToWorld(caliParams,
                                  result->imgPList,
                                  &result->worldPList,
                                  &result->worldAngleList);
    PRINT_ERROR_CODE(ret)


    ret = CRWriteHandPoints(result->worldPList,
                            result->worldAngleList,
                            "worldPoints.csv");
    PRINT_ERROR_CODE(ret)

    // release resource
    CR_Image_Resource_Release(runtime_img);
    CRHandEyeCaliParamRelease(caliParams);
    CRHandEyeResultRelease(result);
    return;
}

int main(int argc, char* argv[])
{
    printf("My pid is: %d\n", getpid());

    handEyeCalibration_test(argc,argv);

    return 0;
}

