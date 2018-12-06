#include "color_palette_generator.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

float ColorPaletteGenerator::maxHueAngle = 270.0f;
int ColorPaletteGenerator::defaultAlpha = 255;

void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, bool reverseRedToBlue) {
    generatePalette(palette, paletteSize, 0, paletteSize);
}

void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseRedToBlue) {
    generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2,0.9f);
}

void ColorPaletteGenerator::generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseRedToBlue){
    double R, G, B;
    double step;

    if(controlPoint1 > controlPoint2 )controlPoint1 = controlPoint2;

    step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);


    int count = 0;
    for (int i = 0; i < controlPoint1; i++) {
        getRGB(step * i, R, G, B, reverseRedToBlue);
        palette[count*4 + 0] = (unsigned char)R;
        palette[count*4 + 1] = (unsigned char)G;
        palette[count*4 + 2] = (unsigned char)B;
        palette[count*4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (controlPoint2Value - controlPoint1Value) / (float)(controlPoint2 - controlPoint1);
    for (int i = 0; i< controlPoint2 - controlPoint1; i++) {
        getRGB(step * i + controlPoint1Value, R, G, B, reverseRedToBlue);
        palette[count * 4 + 0] = (unsigned char)R;
        palette[count * 4 + 1] = (unsigned char)G;
        palette[count * 4 + 2] = (unsigned char)B;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (1.0f - controlPoint2Value) / (float)(paletteSize - controlPoint2);
    for (int i = 0; i < paletteSize - controlPoint2; i++) {
        getRGB(step * i + controlPoint2Value, R, G, B, reverseRedToBlue);
        palette[count * 4 + 0] = (unsigned char)R;
        palette[count * 4 + 1] = (unsigned char)G;
        palette[count * 4 + 2] = (unsigned char)B;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    palette[0] = 0;
    palette[1] = 0;
    palette[2] = 0;
    palette[3] = (unsigned char)defaultAlpha;
    //palette[(paletteSize - 1)*4 + 0] = 255;
    //palette[(paletteSize - 1)*4 + 1] = 255;
    //palette[(paletteSize - 1)*4 + 2] = 255;
    //palette[(paletteSize - 1)*4 + 3] = (unsigned char)defaultAlpha;
}

void ColorPaletteGenerator::generatePaletteColor(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseRedToBlue) {
    float ROI2 = 1.0f;
    float ROI1 = 0.0f;

    //The value ranges from 0.0f ~ 1.0f as hue angle
    float ROI2Value = 1.0f;
    float ROI1Value = 0.0f;

    unsigned char* buf = (unsigned char*)malloc(sizeof(unsigned char) * 4 * size);
    //unsigned char buf[(size) * 4];
    //Set ROI by mode setting.The bigger the disparity the nearer the distance
    switch (mode) {
    case 1: //near
        ROI2 = 0.8f;
        ROI1 = 0.5f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 2: //midle
        ROI2 = 0.7f;
        ROI1 = 0.3f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 3: //far
        ROI2 = 0.6f;
        ROI1 = 0.2f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 4: //custom
        ROI2 = 1.0f*customROI2 / size;
        ROI1 = 1.0f*customROI1 / size;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    default: //normal
        ROI2 = 1.0f;
        ROI1 = 0.0f;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    }

    generatePalette(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseRedToBlue);
    for (int i = 0; i < size; i++) {
        palette[i].rgbBlue     = buf[i * 4 + 2];
        palette[i].rgbGreen    = buf[i * 4 + 1];
        palette[i].rgbRed      = buf[i * 4 + 0];
        palette[i].rgbReserved = buf[i * 4 + 3];
    }
}


void ColorPaletteGenerator::generatePaletteGray(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseGraylevel){
    float ROI1 = 0.0f;
    float ROI2 = 1.0f;


    //The value ranges from 0.0f ~ 1.0f as hue angle
    float ROI1Value = 0.0f;
    float ROI2Value = 1.0f;


    unsigned char* buf = (unsigned char*)malloc(sizeof(unsigned char) * 4 * size);
    //unsigned char buf[(size) * 4];
    //Set ROI by mode setting.The bigger the disparity the nearer the distance
    switch (mode) {
    case 1: //near
        ROI2 = 0.8f;
        ROI1 = 0.5f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 2: //midle
        ROI2 = 0.7f;
        ROI1 = 0.3f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 3: //far
        ROI2 = 0.6f;
        ROI1 = 0.2f;
        ROI2Value = 0.9f;
        ROI1Value = 0.1f;
        break;
    case 4: //custom
        ROI2 = 1.0f*customROI2 / size;
        ROI1 = 1.0f*customROI1 / size;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    default: //normal
        ROI2 = 1.0f;
        ROI1 = 0.0f;
        ROI2Value = 1.0f;
        ROI1Value = 0.0f;
        break;
    }

    generatePaletteGray(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseGraylevel);
    for (int i = 0; i < size; i++) {
        palette[i].rgbBlue = buf[i * 4 + 2];
        palette[i].rgbGreen = buf[i * 4 + 1];
        palette[i].rgbRed = buf[i * 4 + 0];
        palette[i].rgbReserved = buf[i * 4 + 3];
    }
}

void ColorPaletteGenerator::generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseGraylevel) {
    generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2, 0.9f);
}

void ColorPaletteGenerator::generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseGraylevel) {
    float step;
    step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);

    int count = 0;
    for (int i = 0; i < controlPoint1; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - (step * i) : (step * i));
        palette[count * 4 + 0] = (unsigned char)grayValue;
        palette[count * 4 + 1] = (unsigned char)grayValue;
        palette[count * 4 + 2] = (unsigned char)grayValue;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (controlPoint2Value - controlPoint1Value) / (float)(controlPoint2 - controlPoint1);
    for (int i = 0; i< controlPoint2 - controlPoint1; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint1Value)) : ((step * i + controlPoint1Value)));
        palette[count * 4 + 0] = (unsigned char)grayValue;
        palette[count * 4 + 1] = (unsigned char)grayValue;
        palette[count * 4 + 2] = (unsigned char)grayValue;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    step = (1.0f - controlPoint2Value) / (float)(paletteSize - controlPoint2);
    for (int i = 0; i < paletteSize - controlPoint2; i++) {
        int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint2Value)) : ((step * i + controlPoint2Value)));
        palette[count * 4 + 0] = (unsigned char)grayValue;
        palette[count * 4 + 1] = (unsigned char)grayValue;
        palette[count * 4 + 2] = (unsigned char)grayValue;
        palette[count * 4 + 3] = (unsigned char)defaultAlpha;
        count++;
    }

    palette[0] = 0;
    palette[1] = 0;
    palette[2] = 0;
    palette[3] = (unsigned char)defaultAlpha;
    //palette[(paletteSize - 1)*4 + 0] = 255;
    //palette[(paletteSize - 1)*4 + 1] = 255;
    //palette[(paletteSize - 1)*4 + 2] = 255;
    //palette[(paletteSize - 1)*4 + 3] = (unsigned char)defaultAlpha;
}

void ColorPaletteGenerator::getRGB(double value, double &R, double &G, double &B, bool reverseRedToBlue ) {

    if (reverseRedToBlue) value = 1.0f - value;
    HSV_to_RGB(value * maxHueAngle, 1.0f,1.0f,R,G,B);
}

void ColorPaletteGenerator::DmColorMode(RGBQUAD palette[256], int mode,int d8Far, int d8Near)
{

    const int size = 1 << 8; // 8bits = 256
    bool reverseRedtoBlue = true;

    generatePaletteColor(palette, size, mode, d8Far, d8Near, reverseRedtoBlue);
}

void ColorPaletteGenerator::DmGrayMode(RGBQUAD palette[256], int mode,int d8Far, int d8Near)
{

    const int size = 1 << 8; // 8bits = 256
    bool reverseRedtoBlue = false;

    generatePaletteGray(palette, size, mode, d8Far, d8Near, reverseRedtoBlue);
}

#ifdef ENABLE_LONG_DEPTHCOLOR_MAP
void ColorPaletteGenerator::DmColorMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{
    const int size = 1 << 11; // 11 bits = 2048
    bool reverseRedtoBlue = true;
    generatePaletteColor(palette, size, mode, d11Far, d11Near, reverseRedtoBlue);
}

void ColorPaletteGenerator::DmGrayMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{

    const int size = 1 << 11; // 11 bits = 2048
    bool reverseGraylevel = false;
    generatePaletteGray(palette, size, mode, d11Far, d11Near, reverseGraylevel);
}

void ColorPaletteGenerator::DmColorMode14(RGBQUAD palette[16384],float zFar,float zNear)
{
    const int size = 1 << 14; // 14 bits
    bool reverseRedtoBlue = false;
    int mode = 4; //custom
    generatePaletteColor(palette, size,mode, zNear, zFar, reverseRedtoBlue);
}

void ColorPaletteGenerator::DmGrayMode14(RGBQUAD *palette,  float zFar,float zNear)
{
    const int size = 1 << 14; // 14bit 16384
    int mode = 4;//custom
    bool reverseGraylevel = true;
    generatePaletteGray(palette, size, mode, zNear, zFar, reverseGraylevel);
}

#else//not ENABLE_LONG_DEPTHCOLOR_MAP
void ColorPaletteGenerator::DmColorMode11(RGBQUAD pallete[2048], int mode)
{
    float R, G, B;
    for (int i = 0; i < 2048; i++) {
        HSV_to_RGB((double)(i >> 3), 1.0, 1.0, R, G, B);

        pallete[i].rgbBlue = (unsigned char)B;
        pallete[i].rgbGreen = (unsigned char)G;
        pallete[i].rgbRed = (unsigned char)R;
        pallete[i].rgbReserved = 0;
    }
}
void ColorPaletteGenerator::DmColorMode14(RGBQUAD *pColorPaletteZ14, int mode)
{
    int i;
    float R, G, B;
    double fx, fy;
    //
    double fCV = 180;
    int nCenter = 1500;
    double r1 = 0.35;
    double r2 = 0.55;
    //
    for (i = 1; i<16384; i++) {
        if (i == nCenter) {
            fy = fCV;
        }
        else if (i<nCenter) {
            fx = (double)(nCenter - i) / nCenter;
            fy = fCV - pow(fx, r1) * fCV;
        }
        else {
            fx = (double)(i - nCenter) / (16384 - nCenter);
            fy = fCV + pow(fx, r2) * (256 - fCV);
        }
        HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
        pColorPaletteZ14[i].rgbBlue = (unsigned char)B;
        pColorPaletteZ14[i].rgbGreen = (unsigned char)G;
        pColorPaletteZ14[i].rgbRed = (unsigned char)R;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 0;
        pColorPaletteZ14[i].rgbBlue = (unsigned char)0;
        pColorPaletteZ14[i].rgbGreen = (unsigned char)0;
        pColorPaletteZ14[i].rgbRed = (unsigned char)0;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 16383;
        pColorPaletteZ14[i].rgbBlue = (unsigned char)255;
        pColorPaletteZ14[i].rgbGreen = (unsigned char)255;
        pColorPaletteZ14[i].rgbRed = (unsigned char)255;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
}
#endif

void ColorPaletteGenerator::HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B)
{
    double nMax,nMin;
    double fDet;
    //
    while (H<0.0) H+=360.0;
    while (H>=360.0) H-=360.0;
    H /= 60.0;
    if (V<0.0) V = 0.0;
    if (V>1.0) V = 1.0;
    V *= 255.0;
    if (S<0.0) S = 0.0;
    if (S>1.0) S = 1.0;
    //
    if (V == 0.0) {
        R = G = B = 0;
    } else {
        fDet = S*V;
        nMax = (V);
        nMin = (V-fDet);
        if (H<=1.0) { //R>=G>=B, H=(G-B)/fDet
            R = nMax;
            B = nMin;
            G = (H*fDet+B);
        } else if (H<=2.0) { //G>=R>=B, H=2+(B-R)/fDet
            G = nMax;
            B = nMin;
            R = ((2.0-H)*fDet+B);
        } else if (H<=3.0) { //G>=B>=R, H=2+(B-R)/fDet
            G = nMax;
            R = nMin;
            B = ((H-2.0)*fDet+R);
        } else if (H<=4.0) { //B>=G>=R, H=4+(R-G)/fDet
            B = nMax;
            R = nMin;
            G = ((4.0-H)*fDet+R);
        } else if (H<=5.0) { //B>=R>=G, H=4+(R-G)/fDet
            B = nMax;
            G = nMin;
            R = ((H-4.0)*fDet+G);
        } else { // if(H<6.0) //R>=B>=G, H=(G-B)/fDet+6
            R = nMax;
            G = nMin;
            B = ((6.0-H)*fDet+G);
        }
    }
}


void ColorPaletteGenerator::SetBaseColorPaletteD11(RGBQUAD *pColorPaletteD11)
{
    int i;
    double R,G,B;
    //
    for (i=0; i<2048; i++) {
        HSV_to_RGB((2047.0-i)/8,1.0,1.0,R,G,B);
        pColorPaletteD11[i].rgbBlue     = (unsigned char)B;
        pColorPaletteD11[i].rgbGreen    = (unsigned char)G;
        pColorPaletteD11[i].rgbRed      = (unsigned char)R;
        pColorPaletteD11[i].rgbReserved = 0;
    }
    {
        i = 0;
        pColorPaletteD11[i].rgbBlue     = (unsigned char)0;
        pColorPaletteD11[i].rgbGreen    = (unsigned char)0;
        pColorPaletteD11[i].rgbRed      = (unsigned char)0;
        pColorPaletteD11[i].rgbReserved = 0;
    }
    {
        i = 2047;
        pColorPaletteD11[i].rgbBlue     = (unsigned char)255;
        pColorPaletteD11[i].rgbGreen    = (unsigned char)255;
        pColorPaletteD11[i].rgbRed      = (unsigned char)255;
        pColorPaletteD11[i].rgbReserved = 0;
    }
}

void ColorPaletteGenerator::SetBaseGrayPaletteD11(RGBQUAD *pGrayPaletteD11)
{
    int i;
    double R,G,B;
    //
    for (i=0; i<2048; i++) {
        HSV_to_RGB((2047.0-i)/8,1.0,1.0,R,G,B);
        pGrayPaletteD11[i].rgbBlue     = (unsigned char)B;
        pGrayPaletteD11[i].rgbGreen    = (unsigned char)B;
        pGrayPaletteD11[i].rgbRed      = (unsigned char)B;
        pGrayPaletteD11[i].rgbReserved = 0;
    }
    {
        i = 0;
        pGrayPaletteD11[i].rgbBlue     = (unsigned char)0;
        pGrayPaletteD11[i].rgbGreen    = (unsigned char)0;
        pGrayPaletteD11[i].rgbRed      = (unsigned char)0;
        pGrayPaletteD11[i].rgbReserved = 0;
    }
    {
        i = 2047;
        pGrayPaletteD11[i].rgbBlue     = (unsigned char)255;
        pGrayPaletteD11[i].rgbGreen    = (unsigned char)255;
        pGrayPaletteD11[i].rgbRed      = (unsigned char)255;
        pGrayPaletteD11[i].rgbReserved = 0;
    }
}

void ColorPaletteGenerator::SetBaseColorPaletteZ14(RGBQUAD *pColorPaletteZ14)
{
    int i;
    double R,G,B;
    double fx,fy;
    //
    double fCV = 180;
    int nCenter=1500;
    double r1=0.35;
    double r2=0.55;
    //
    for (i=1; i<16384; i++) {
        if (i==nCenter) {
            fy = fCV;
        } else if (i<nCenter) {
            fx = (double)(nCenter-i)/nCenter;
            fy = fCV - pow(fx, r1)*fCV;
        } else {
            fx = (double)(i-nCenter)/(16384-nCenter);
            fy = fCV + pow(fx, r2)*(256-fCV);
        }
        HSV_to_RGB(fy,1.0,1.0,R,G,B);
        pColorPaletteZ14[i].rgbBlue     = (unsigned char)B;
        pColorPaletteZ14[i].rgbGreen    = (unsigned char)G;
        pColorPaletteZ14[i].rgbRed      = (unsigned char)R;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 0;
        pColorPaletteZ14[i].rgbBlue     = (unsigned char)0;
        pColorPaletteZ14[i].rgbGreen    = (unsigned char)0;
        pColorPaletteZ14[i].rgbRed      = (unsigned char)0;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 16383;
        pColorPaletteZ14[i].rgbBlue     = (unsigned char)255;
        pColorPaletteZ14[i].rgbGreen    = (unsigned char)255;
        pColorPaletteZ14[i].rgbRed      = (unsigned char)255;
        pColorPaletteZ14[i].rgbReserved = 0;
    }
}

void ColorPaletteGenerator::ColorPaletteGenerator::SetBaseGrayPaletteZ14(RGBQUAD *pGrayPaletteZ14)
{
    int i;
    double R,G,B;
    double fx,fy;
    //
    double fCV = 180;
    int nCenter=1500;
    double r1=0.35;
    double r2=0.55;
    //
    for (i=1; i<16384; i++) {
        if (i==nCenter) {
            fy = fCV;
        } else if (i<nCenter) {
            fx = (double)(nCenter-i)/nCenter;
            fy = fCV - pow(fx, r1)*fCV;
        } else {
            fx = (double)(i-nCenter)/(16384-nCenter);
            fy = fCV + pow(fx, r2)*(256-fCV);
        }
        HSV_to_RGB(fy,1.0,1.0,R,G,B);
        pGrayPaletteZ14[i].rgbBlue     = (unsigned char)B;
        pGrayPaletteZ14[i].rgbGreen    = (unsigned char)B;
        pGrayPaletteZ14[i].rgbRed      = (unsigned char)B;
        pGrayPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 0;
        pGrayPaletteZ14[i].rgbBlue     = (unsigned char)0;
        pGrayPaletteZ14[i].rgbGreen    = (unsigned char)0;
        pGrayPaletteZ14[i].rgbRed      = (unsigned char)0;
        pGrayPaletteZ14[i].rgbReserved = 0;
    }
    {
        i = 16383;
        pGrayPaletteZ14[i].rgbBlue     = (unsigned char)255;
        pGrayPaletteZ14[i].rgbGreen    = (unsigned char)255;
        pGrayPaletteZ14[i].rgbRed      = (unsigned char)255;
        pGrayPaletteZ14[i].rgbReserved = 0;
    }
}

void ColorPaletteGenerator::UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette8bits, unsigned char *pDepth8bits, unsigned char *pDepthDIB24, int cx, int cy)
{
    int x,y,nBPS;
    unsigned char *pWSL,*pWS;
    unsigned char *pDL,*pD;
    RGBQUAD *pClr;
    //
    if ((cx <= 0) || (cy <= 0)) return;

    nBPS = cx * 3;
    pWSL = pDepth8bits;
    pDL = pDepthDIB24;
    for (y = 0; y < cy; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x = 0; x < cx; x++) {
            pClr = &(pColorPalette8bits[pWS[x]]);

            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;

            pD += 3;
        }
        pWSL += cx;
        pDL += nBPS;
    }
}


void ColorPaletteGenerator::UpdateD11DisplayImage_DIB24(RGBQUAD *pColorPaletteD11, unsigned char *pDepthD11, unsigned char *pDepthDIB24, int cx, int cy)
{
    int x,y,nBPS;
    unsigned short *pWSL,*pWS;
    unsigned char *pDL,*pD;
    RGBQUAD *pClr;
    //
    if ((cx <= 0) || (cy <= 0)) return;

    nBPS = cx * 3;
    pWSL = (unsigned short *)pDepthD11;
    pDL = pDepthDIB24;
    for (y = 0; y < cy; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x = 0; x < cx; x++) {
            pClr = &(pColorPaletteD11[pWS[x]]);

            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;

            pD += 3;
        }
        pWSL += cx;
        pDL += nBPS;
    }
}

void ColorPaletteGenerator::UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14, unsigned char *pDepthDIB24, int cx, int cy)
{
    int x,y,nBPS;
    unsigned short *pWSL,*pWS;
    unsigned char *pDL,*pD;
    RGBQUAD *pClr;
    //
    if ((cx <= 0) || (cy <= 0)) return;
    //
    nBPS = cx * 3;
    pWSL = (unsigned short *)pDepthZ14;
    pDL = pDepthDIB24;
    for (y=0; y<cy; y++) {
        pWS = pWSL;
        pD = pDL;
        for (x=0; x<cx; x++) {
            pClr = &(pColorPaletteZ14[pWS[x]]);
            pD[0] = pClr->rgbRed;
            pD[1] = pClr->rgbGreen;
            pD[2] = pClr->rgbBlue;
            pD += 3;
        }
        pWSL += cx;
        pDL += nBPS;
    }
}
