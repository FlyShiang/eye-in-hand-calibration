/*
 * Author: Yu-Shiang Yan
 *
 * Note: Utility Interface(Ex. read CSV file, write CSV file)
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <Utility.h>
#include <Image_Params.h>

int CRChessBoardTesting(std::vector<Point2f>* imgPointList,
                        std::string           imgFileName)
{
    int exitCode = dFALSE;

    Mat frame;

    frame = imread(imgFileName);

    exitCode = CRAlgoTestChessBoard(frame,
                                    imgPointList);

    frame.release();

    if(exitCode)
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_TESTING_CHESS_BOARD;
    }

    return exitCode;

}

int CRWriteImgPoints(std::vector<cv::Point2f>   imagePoints,
                     std::string                filename)
{
    int i;

    std::ofstream myfile;
    myfile.open (filename);

    for(i = 0; i < (int)imagePoints.size(); i++)
    {
       myfile << i+1 << "," << imagePoints[i].x << "," << imagePoints[i].y << std::endl;
    }

    myfile.close();

    return 0;
}

int CRWriteHandPoints(std::vector<Point3f>      imgToHandPoints,
                      std::vector<CRAngleInfo>  imgToHandAnlge,
                      std::string               filename)
{
    int i;

    std::ofstream myfile;
    myfile.open (filename);

    myfile << "Index" << ","
           << "Name"  << ","
           << "X"  << ","
           << "Y"  << ","
           << "Z"  << ","
           << "RX" << ","
           << "RY" << ","
           << "RZ" << ","
           << "Elbow" << ","
           << "Shoulder" << ","
           << "Filp" << ","
           << "UserID" << ","
           << "ToolID" << ","
           << "Coordinate" << ","
           << "J4_JRC" << ","
           << "J6_JRC" << ","
           << "JRC_Active" << "," << "\n";

    for(i = 0; i < (int)imgToHandPoints.size() * 3; i++)
    {
        if((i % 3) == 0)
        {
            myfile << i+1001 << ","
                  << "p100" << i+1  << ","
                  << imgToHandPoints[i/3].x << ","
                  << imgToHandPoints[i/3].y << ","
                  << imgToHandPoints[i/3].z + 50 << ","
                  << imgToHandAnlge[i/3].rx << ","
                  << imgToHandAnlge[i/3].ry << ","
                  << imgToHandAnlge[i/3].rz << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "1" << ","
                  << "0" << ","
                  << "0" << ","
                  << "1" << ","
                  << "\n";
        }
        else if((i % 3) == 1)
        {
            myfile << i+1001 << ","
                  << "p100" << i+1  << ","
                  << imgToHandPoints[i/3].x << ","
                  << imgToHandPoints[i/3].y << ","
                  << imgToHandPoints[i/3].z << ","
                  << imgToHandAnlge[i/3].rx << ","
                  << imgToHandAnlge[i/3].ry << ","
                  << imgToHandAnlge[i/3].rz << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "0" << ","
                  << "1" << ","
                  << "0" << ","
                  << "0" << ","
                  << "1" << ","
                  << "\n";
        }
        else if((i % 3) == 2)
        {
          myfile << i+1001 << ","
                << "p100" << i+1  << ","
                << imgToHandPoints[i/3].x << ","
                << imgToHandPoints[i/3].y << ","
                << imgToHandPoints[i/3].z + 50 << ","
                << imgToHandAnlge[i/3].rx << ","
                << imgToHandAnlge[i/3].ry << ","
                << imgToHandAnlge[i/3].rz << ","
                << "0" << ","
                << "0" << ","
                << "0" << ","
                << "0" << ","
                << "0" << ","
                << "1" << ","
                << "0" << ","
                << "0" << ","
                << "1" << ","
                << "\n";
        }
    }

    myfile.close();

    return 0;
}

int CRReadCalibrationHandInfo(std::vector<CRHandInfo>*      calibrationHandInfo,
                              std::string                   fileName,
                              int                           CalibrationImgNum)
{
    int i   = 0,
        j   = 0;

    float temp[6] = {0};

    std::ifstream inFile;

    std::string tempStr;

    #define CR_HAND_INFO_EXCEL_INDEX_NUM 17
    #define CR_HAND_INFO_START_INDEX 2
    #define CR_HAND_INFO_END_INDEX 7

    inFile.open(fileName);

    if (!inFile.is_open())
    {
        std::cout << "NO FILE HAS BEEN OPENED" << std::endl;
        return dTRUE;
    }

    for(i = 0; i < CalibrationImgNum + 1; i++)
    {
        if(i == 0)
        {
            for(j = 0; j < CR_HAND_INFO_EXCEL_INDEX_NUM; j++)
            {
                if(j == CR_HAND_INFO_EXCEL_INDEX_NUM -1)
                {
                    getline(inFile,tempStr, '\n');
                }
                else
                {
                    getline(inFile,tempStr, ',');
                }
            }
        }
        else
        {
            for(j = 0; j < CR_HAND_INFO_EXCEL_INDEX_NUM; j++)
            {
                if(j == CR_HAND_INFO_EXCEL_INDEX_NUM -1)
                {
                    getline(inFile,tempStr, '\n');
                }
                else
                {
                    getline(inFile,tempStr, ',');
                }

                if(j >= CR_HAND_INFO_START_INDEX && j <= CR_HAND_INFO_END_INDEX)
                {
                    temp[j-CR_HAND_INFO_START_INDEX] = atof(tempStr.c_str());
                }
            }

            CRHandInfo handInfo;

            handInfo.x = temp[0];
            handInfo.y = temp[1];
            handInfo.z = temp[2];
            handInfo.rx = temp[3];
            handInfo.ry = temp[4];
            handInfo.rz = temp[5];

            calibrationHandInfo->push_back(handInfo);
        }
    }

    inFile.close();

    #undef CR_HAND_INFO_EXCEL_INDEX_NUM
    #undef CR_HAND_INFO_START_INDEX
    #undef CR_HAND_INFO_END_INDEX

    return 0;
}

static void CRReadCSVToRmatParams(Mat& param,
                                  std::ifstream& myfile)
{
    int i   = 0,
        j   = 0;

    std::string tempStr;

    getline(myfile,tempStr, ',');

    for(j = 0; j < 3; j++)
    {
        for(i = 0; i < 3; i++)
        {
            getline(myfile,tempStr, ',');

            param.at<double>(j,i)= atof(tempStr.c_str());
        }
    }

    getline(myfile,tempStr, '\n');
}

static void CRReadCSVToTvecParams(Mat& param,
                                  std::ifstream& myfile)
{
    int i   = 0;

    std::string tempStr;

    getline(myfile,tempStr, ',');

    for(i = 0; i < 3; i++)
    {
        getline(myfile,tempStr, ',');

        param.at<double>(i,0)= (double)atof(tempStr.c_str());
    }

    getline(myfile,tempStr, '\n');
}

int CRReadCalibrationParams(CR_EyeInHand_Cali_Param*      pCaliInfo,
                            std::string                   fileName)
{
    int exitCode = dFALSE;

    std::ifstream inFile;

    inFile.open(fileName);

    if (!inFile.is_open())
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_READ_PARAMS;
    }

    CRReadCSVToRmatParams(pCaliInfo->oMiMatrix,
                          inFile);

    CRReadCSVToRmatParams(pCaliInfo->cMoRmatrix,
                          inFile);

    CRReadCSVToRmatParams(pCaliInfo->eMcRmatrix,
                          inFile);

    CRReadCSVToRmatParams(pCaliInfo->wMeShotRmatrix,
                          inFile);

    CRReadCSVToRmatParams(pCaliInfo->wMeTouchRmatrix,
                          inFile);

    CRReadCSVToTvecParams(pCaliInfo->tvecsCamObject,
                          inFile);

    CRReadCSVToTvecParams(pCaliInfo->tvecsEndEffectorCam,
                          inFile);

    CRReadCSVToTvecParams(pCaliInfo->tvecsShotWorldEndEffector,
                          inFile);

    CRReadCSVToTvecParams(pCaliInfo->tvecsEndEffectorTool,
                          inFile);
    inFile.close();

    return exitCode;
}

static void CRWriteRmatParamsToCSV(Mat param,
                                  std::string paramName,
                                  std::ofstream& myfile)
{
    int i = 0,
        j = 0;

    myfile << paramName << ',';

    for(j = 0; j < 3; j++)
    {
        for(i = 0; i < 3; i++)
        {
            myfile << param.at<double>(j,i) << ',';
        }
    }

    myfile << std::endl;
}

static void CRWriteTvecParamsToCSV(Mat param,
                                  std::string paramName,
                                  std::ofstream& myfile)
{
    int i = 0;

    myfile << paramName << ',';


    for(i = 0; i < 3; i++)
    {
       myfile << param.at<double>(i,0) << ',';
    }

    myfile << std::endl;
}


int CRWriteCalibrationParams(CR_EyeInHand_Cali_Param*    pCaliInfo,
                             std::string                 fileName)
{
    int exitCode = dFALSE;

    std::ofstream myfile;

    myfile.open (fileName);

    if (!myfile.is_open())
    {
        return eIMAGE_PROCESS_HAND_EYE_CALIBRATION_CALC_WRITE_PARAMS;
    }

    CRWriteRmatParamsToCSV(pCaliInfo->oMiMatrix,
                          "oMiMatrix",
                          myfile);

    CRWriteRmatParamsToCSV(pCaliInfo->cMoRmatrix,
                          "cMoRmatrix",
                          myfile);

    CRWriteRmatParamsToCSV(pCaliInfo->eMcRmatrix,
                          "eMcRmatrix",
                          myfile);

    CRWriteRmatParamsToCSV(pCaliInfo->wMeShotRmatrix,
                          "wMeShotRmatrix",
                          myfile);

    CRWriteRmatParamsToCSV(pCaliInfo->wMeTouchRmatrix,
                          "wMeTouchRmatrix",
                          myfile);


    CRWriteTvecParamsToCSV(pCaliInfo->tvecsCamObject,
                           "tvecsCamObject",
                           myfile);

    CRWriteTvecParamsToCSV(pCaliInfo->tvecsEndEffectorCam,
                           "tvecsEndEffectorCam",
                           myfile);

    CRWriteTvecParamsToCSV(pCaliInfo->tvecsShotWorldEndEffector,
                           "tvecsShotWorldEndEffector",
                           myfile);

    CRWriteTvecParamsToCSV(pCaliInfo->tvecsEndEffectorTool,
                           "tvecsEndEffectorTool",
                           myfile);

    myfile.close();

    return exitCode;
}

int CRAlgoCVSaveAsImg(unsigned char*    raw_data,
                      int             	img_w,
                      int             	img_h,
                      int             	img_pitch,
                      char*           	img_str)
{
    int             exitCode = dFALSE;
    std::string          str(img_str);

    try
    {
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

int CRAlgoCVReadImg(unsigned char*		raw_data,
                    int*				img_w,
                    int*				img_h,
                    int*				img_pitch,
                    char*				img_str)
{
    int             exitCode = dFALSE,
                    i = 0,
                    j = 0;
    const std::string    str(img_str);

    try
    {
        Mat img;

        img = imread(str,IMREAD_GRAYSCALE );

        *img_h = img.rows;
        *img_w = img.cols;
        *img_pitch = img.cols;

        for(j = 0; j < img.rows; j++)
        {
            for(i = 0; i < img.cols; i++)
            {
               raw_data[j * img.cols + i] = img.at<uchar>(j,i);
            }
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
