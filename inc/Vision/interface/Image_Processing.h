/*
 * Author: Yu-Shiang Yan
 *
 * Note: Image processing interface
 */
 
#ifndef IMAGE_PROCESSING_H_INCLUDED
#define IMAGE_PROCESSING_H_INCLUDED

#include <ImageAlgo_HandEye_Calibration.h>

#ifdef __cplusplus
extern "C" {
#endif

/* image */
typedef struct struct_CR_Image
{
    unsigned char	    *raw_data;
    unsigned char           *mask_data;
    int             	    width;
    int             	    height;
    int			    pitch;
    int                     roi_shape;

} CR_Image;

/* eye in hand calibration parameters */
class CR_EyeInHand_Cali_Param {
public:
    Mat oMiMatrix;
    Mat cMoRmatrix;
    Mat eMcRmatrix;
    Mat wMeShotRmatrix;
    Mat wMeTouchRmatrix;
    Mat tvecsCamObject;
    Mat tvecsEndEffectorCam;
    Mat tvecsShotWorldEndEffector;
    Mat tvecsTouchWorldEndEffector;
    Mat tvecsEndEffectorTool;
    Size imgResolution;
    std::vector <CRHandInfo> calibrationHandInfo;
    std::vector<std::vector<Point2f>> calibrationImgFeature;
    CRHandInfo shotInitHandInfo;
    CRHandInfo shotRunTimeHandInfo;
    CRHandInfo touchHandInfo;
    int    gridSize;
    double toolWidth;
    double toolHeight;
    double toolTheta;
};

class CR_EyeInHand_Result {
    public:
    std::vector<Point2f> imgPList;
    std::vector<Point3f> worldPList;
    std::vector<CRAngleInfo> worldAngleList;
};

/**
 * @brief   read image
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @param	img_str     [in]    image file name
 * @return	error code  [out]
 */
int CR_ReadImg(CR_Image*   srcImg,
               char*       str);

/**
 * @brief   save as image
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @param	img_str     [in]    image file name
 * @return	error code  [out]
 */			
int CR_SaveAsImg(CR_Image*  image,
                 char*      img_str);

					  
/**
 * @brief   initialize image resoure(image's memory)
 * @param	image       [in]    image structure(raw data, width, etc.)
 * @return	error code  [out]
 */
int CR_Image_Resource_Init(CR_Image** image);

/**
 * @brief   release image resoure
 * @param	image   [in]    image structure(raw data, width, etc.)
 */
void CR_Image_Resource_Release(CR_Image* image);



void CRHandEyeResultClear(CR_EyeInHand_Result *result);

int CRHandEyeResultInit(CR_EyeInHand_Result **result);

void CRHandEyeResultRelease(CR_EyeInHand_Result *result);

int CRHandEyeCaliParamInit(CR_EyeInHand_Cali_Param **caliParam);

void CRHandEyeCaliParamRelease(CR_EyeInHand_Cali_Param *caliParam);

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
                                CR_EyeInHand_Cali_Param*  pCaliInfo,
                                bool           *found);

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
                            CR_EyeInHand_Cali_Param*  pCaliInfo);

int CRHandEyeGetToolInfo(double     toolWidth,
                         double     toolHeight,
                         double     toolTheta,
                         CR_EyeInHand_Cali_Param*  pCaliInfo);


int CRHandEyeCalcCalibration(CR_EyeInHand_Cali_Param*  pCaliInfo);


int CRHandEyeCalcImgToWorld(CR_EyeInHand_Cali_Param*        pCaliInfo,
                            std::vector<Point2f>            imgPointList,
                            std::vector<Point3f>*           worldPointList,
                            std::vector<CRAngleInfo>*       worldAngleList);


void CRHandEyeParamsClear(CR_EyeInHand_Cali_Param*        pCaliInfo);


#ifdef __cplusplus
}
#endif

#endif //
