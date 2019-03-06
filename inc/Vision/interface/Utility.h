/*
 * Author: Yu-Shiang Yan
 *
 * Note: Utility Interface(Ex. read CSV file, write CSV file)
 */
 
#ifndef UTILITY_H_INCLUDED
#define UTILITY_H_INCLUDED

#include <Image_Processing.h>

#ifdef __cplusplus
extern "C" {
#endif

int CRChessBoardTesting(std::vector<Point2f>*   imgPointList,
                        std::string             imgFileName);



int CRReadCalibrationHandInfo(std::vector <CRHandInfo>*     calibrationHandInfo,
                              std::string                   fileName,
                              int                           CalibrationImgNum);


int CRReadCalibrationParams(CR_EyeInHand_Cali_Param*         pCaliInfo,
                            std::string                      fileName);


int CRWriteImgPoints(std::vector<Point2f>   imagePoints,
                     std::string            filename);

int CRWriteHandPoints(std::vector<Point3f>      imgToHandPoints,
                      std::vector<CRAngleInfo>  imgToHandAnlge,
                      std::string               filename);


int CRWriteCalibrationParams(CR_EyeInHand_Cali_Param*    pCaliInfo,
                             std::string                 fileName);

int CRAlgoCVSaveAsImg(unsigned char*    raw_data,
                      int             	img_w,
                      int             	img_h,
                      int             	img_pitch,
                      char*           	img_str);

int CRAlgoCVReadImg(unsigned char*		raw_data,
                    int*			img_w,
                    int*			img_h,
                    int*			img_pitch,
                    char*			img_str);

#ifdef __cplusplus
}
#endif

#endif //
