#include "colorizer_win.h"

#include "color_palette_generator.h"

namespace {

#define Z14_NEAR 0
#define D11_FAR  0
#define D11_NEAR 2047
#define D8_FAR   0
#define D8_NEAR  255

void generatePaletteColor(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseRedToBlue) {
	float ROI2 = 1.0f;
	float ROI1 = 0.0f;

	//The value ranges from 0.0f ~ 1.0f as hue angle
	float ROI2Value = 1.0f;
	float ROI1Value = 0.0f;

	BYTE* buf = (BYTE*)malloc(sizeof(BYTE) * 4 * size);
	//BYTE buf[(size) * 4];
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

	ColorPaletteGenerator::generatePalette(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseRedToBlue);
	for (int i = 0; i < size; i++) {
		palette[i].rgbBlue		= buf[i * 4 + 2];
		palette[i].rgbGreen		= buf[i * 4 + 1];
		palette[i].rgbRed		= buf[i * 4 + 0];
		palette[i].rgbReserved	= buf[i * 4 + 3];
	}
}


void generatePaletteGray(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseGraylevel){
	float ROI1 = 0.0f;
	float ROI2 = 1.0f;


	//The value ranges from 0.0f ~ 1.0f as hue angle
	float ROI1Value = 0.0f;
	float ROI2Value = 1.0f;


	BYTE* buf = (BYTE*)malloc(sizeof(BYTE) * 4 * size);
	//BYTE buf[(size) * 4];
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

	ColorPaletteGenerator::generatePaletteGray(buf, size, ROI1 * size, ROI1Value, ROI2 * size, ROI2Value, reverseGraylevel);
	for (int i = 0; i < size; i++) {
		palette[i].rgbBlue = buf[i * 4 + 2];
		palette[i].rgbGreen = buf[i * 4 + 1];
		palette[i].rgbRed = buf[i * 4 + 0];
		palette[i].rgbReserved = buf[i * 4 + 3];
	}
}

void DmColorMode(unsigned char palette[256][4], int mode,int d8Far, int d8Near)
{

	const int size = 1 << 8; // 8bits = 256
	bool reverseRedtoBlue = true;
	RGBQUAD RGBpalette[size];
	generatePaletteColor(RGBpalette, size, mode, d8Far, d8Near, reverseRedtoBlue);

	for (int i = 0; i < size; i++) {
		palette[i][0] = RGBpalette[i].rgbBlue;
		palette[i][1] = RGBpalette[i].rgbGreen;
		palette[i][2] = RGBpalette[i].rgbRed;
		palette[i][3] = RGBpalette[i].rgbReserved;
	}
}
#ifdef ENABLE_LONG_DEPTHCOLOR_MAP
void DmColorMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{
	const int size = 1 << 11; // 11 bits = 2048
	bool reverseRedtoBlue = true;
	generatePaletteColor(palette, size, mode, d11Far, d11Near, reverseRedtoBlue);
}

void DmGrayMode11(RGBQUAD palette[2048], int mode,int d11Far,int d11Near)
{

	const int size = 1 << 11; // 11 bits = 2048
	bool reverseGraylevel = false;
	generatePaletteGray(palette, size, mode, d11Far, d11Near, reverseGraylevel);
}
void DmColorMode14(RGBQUAD palette[16384],float zFar,float zNear)
{
	const int size = 1 << 14; // 14 bits
	bool reverseRedtoBlue = false;
	int mode = 4; //custom
	generatePaletteColor(palette, size,mode, zNear, zFar, reverseRedtoBlue);
}

void DmGrayMode14(RGBQUAD *palette,  float zFar,float zNear)
{
	const int size = 1 << 14; // 14bit 16384
	int mode = 4;//custom
	bool reverseGraylevel = true;
	generatePaletteGray(palette, size, mode, zNear, zFar, reverseGraylevel);
}
#else//not ENABLE_LONG_DEPTHCOLOR_MAP
// k= 0~1.00
// maps k to a pixel color RGB
void ColorMap(double k, double& R, double& G, double& B) {  // NOLINT
  double r;
  if (k < 0.0) k = 0.0;
  if (k > 1.0) k = 1.0;
  if (k < 0.1) {
    r = k / 0.1;
    R = G = B = 128.0 + r * 127.0;  // 128~255
  } else if (k < 0.2) {
    k -= .1;
    r = k / 0.1;
    R = 255.0;
    G = B = (1.0 - r) * 255.0;  // 255~0
  } else if (k < 0.35) {
    k -= .2;
    r = k / 0.15;
    B = 0.0;  // B
    G = r * 255.0;  // 0~255
    R = 255.0;  // R
  } else if (k < 0.5) {
    k -= 0.35;
    r = k / 0.15;
    B = 0.0;
    G = (1.0 - r / 4.0) * 255.0;  // 255~196
    R = (1.0 - r / 2.0) * 255.0;  // 255~128
  } else if (k < 0.6) {
    k -= 0.5;
    r = k / 0.1;
    B = r * 128.0;  // B 0~128
    G = 196.0;  // G
    R = (1.0 - r) * 128.0;  // R 128~0
  } else if (k < 0.7) {
    k -= 0.6;
    r = k / 0.1;
    B = 128.0 + r * 127.0;  // B 128~255
    G = 196.0;  // G
    R = 0.0;  // R
  } else if (k < 0.8) {
    k -= 0.7;
    r = k / 0.1;
    B = 255;  // B
    G = (1.0 - r) * 196.0;  // G 196~0
    R = 0;  // R
  } else if (k < 0.9) {
    k -= 0.8;
    r = k / 0.1;
    B = (1.0 - r / 2.0) * 255.0;  // B 255~128
    G = 0.0;  // G
    R = r * 128.0;  // R=0~128
  } else {
    k -= .9;
    r = k / .1;
    R = B = (1 - r) * 128;  // B 128~0
    G = 0;  // G
  }
}

void HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B) {
  double nMax, nMin;
  double fDet;
  //
  while (H < 0.0) H += 360.0;
  while (H >= 360.0) H -= 360.0;
  H /= 60.0;
  if (V < 0.0) V = 0.0;
  if (V > 1.0) V = 1.0;
  V *= 255.0;
  if (S < 0.0) S = 0.0;
  if (S > 1.0) S = 1.0;
  //
  if (V == 0.0) {
    R = G = B = 0;
  } else {
    fDet = S*V;
    nMax = (V);
    nMin = (V-fDet);
    if (H <= 1.0) { // R>=G>=B, H=(G-B)/fDet
      R = nMax;
      B = nMin;
      G = (H*fDet+B);
    } else if (H <= 2.0) { // G>=R>=B, H=2+(B-R)/fDet
      G = nMax;
      B = nMin;
      R = ((2.0-H)*fDet+B);
    } else if (H <= 3.0) { // G>=B>=R, H=2+(B-R)/fDet
      G = nMax;
      R = nMin;
      B = ((H-2.0)*fDet+R);
    } else if (H <= 4.0) { // B>=G>=R, H=4+(R-G)/fDet
      B = nMax;
      R = nMin;
      G = ((4.0-H)*fDet+R);
    } else if (H <= 5.0) { // B>=R>=G, H=4+(R-G)/fDet
      B = nMax;
      G = nMin;
      R = ((H-4.0)*fDet+G);
    } else { // if(H<6.0) // R>=B>=G, H=(G-B)/fDet+6
      R = nMax;
      G = nMin;
      B = ((6.0-H)*fDet+G);
    }
  }
}

void SetBaseGrayPaletteZ14(RGBQUAD *pGrayPaletteZ14) {
  int i;
  double R, G, B;
  double fx, fy;
  //
  double fCV = 180;
  int nCenter = 1500;
  double r1 = 0.35;
  double r2 = 0.55;
  //
  for (i = 1; i < 16384; i++) {
    if (i == nCenter) {
      fy = fCV;
    } else if (i < nCenter) {
      fx = (double)(nCenter - i) / nCenter;
      fy = fCV - pow(fx, r1)*fCV;
    } else {
      fx = (double)(i - nCenter) / (16384 - nCenter);
      fy = fCV + pow(fx, r2)*(256-fCV);
    }
    HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)B;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)B;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)B;
    pGrayPaletteZ14[i].rgbReserved = 0;
  }
  {
    i = 0;
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)0;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)0;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)0;
    pGrayPaletteZ14[i].rgbReserved = 0;
  }
  {
    i = 16383;
    pGrayPaletteZ14[i].rgbBlue     = (BYTE)255;
    pGrayPaletteZ14[i].rgbGreen    = (BYTE)255;
    pGrayPaletteZ14[i].rgbRed      = (BYTE)255;
    pGrayPaletteZ14[i].rgbReserved = 0;
  }
}

void DmColorMode11(RGBQUAD pallete[2048], int mode)
{
	double R, G, B;
	for (int i = 0; i<2048; i++) {
		HSV_to_RGB((double)(i >> 3), 1.0, 1.0, R, G, B);

		pallete[i].rgbBlue = (BYTE)B;
		pallete[i].rgbGreen = (BYTE)G;
		pallete[i].rgbRed = (BYTE)R;
		pallete[i].rgbReserved = 0;
	}
}
void DmColorMode14(RGBQUAD *pColorPaletteZ14, int mode)
{
	int i;
	double R, G, B;
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
			fy = fCV - pow(fx, r1)*fCV;
		}
		else {
			fx = (double)(i - nCenter) / (16384 - nCenter);
			fy = fCV + pow(fx, r2)*(256 - fCV);
		}
		HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
		pColorPaletteZ14[i].rgbBlue = (BYTE)B;
		pColorPaletteZ14[i].rgbGreen = (BYTE)G;
		pColorPaletteZ14[i].rgbRed = (BYTE)R;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
	{
		i = 0;
		pColorPaletteZ14[i].rgbBlue = (BYTE)0;
		pColorPaletteZ14[i].rgbGreen = (BYTE)0;
		pColorPaletteZ14[i].rgbRed = (BYTE)0;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
	{
		i = 16383;
		pColorPaletteZ14[i].rgbBlue = (BYTE)255;
		pColorPaletteZ14[i].rgbGreen = (BYTE)255;
		pColorPaletteZ14[i].rgbRed = (BYTE)255;
		pColorPaletteZ14[i].rgbReserved = 0;
	}
}
#endif //ENABLE_LONG_DEPTHCOLOR_MAP

void UpdateZ14DisplayImage_DIB24(RGBQUAD* pColorPaletteZ14, BYTE* pDepthZ14,
    BYTE* pDepthDIB24, int cx, int cy) {
  int x, y, nBPS;
  WORD *pWSL, *pWS;
  BYTE *pDL, *pD;
  RGBQUAD *pClr;

  if ((cx <= 0) || (cy <= 0)) return;

  nBPS = ((cx*3+3)/4)*4;
  pWSL = (WORD*)pDepthZ14;  // NOLINT
  // pDL = pDepthDIB24 + (cy-1)*nBPS;
  pDL = pDepthDIB24;
  for (y = 0; y < cy; y++) {
    pWS = pWSL;
    pD = pDL;
    for (x = 0; x < cx; x++) {
      pClr = &(pColorPaletteZ14[pWS[x]]);
      pD[0] = pClr->rgbBlue;  // B
      pD[1] = pClr->rgbGreen;  // G
      pD[2] = pClr->rgbRed;  // R
      pD += 3;
    }
    pWSL += cx;
    // pDL -= nBPS;
    pDL += nBPS;
  }
}

}  // namespace

MYNTEYE_BEGIN_NAMESPACE

void ColorizerWin::Init(float z14_far,
    bool is_8bits,
    std::shared_ptr<CameraCalibration> calib_params) {
  ColorizerPrivate::Init(z14_far, is_8bits, calib_params);

  float m_zFar = z14_far;
  float m_zNear = Z14_NEAR;
  int m_d11Far = D11_FAR;
  int m_d11Near = D11_NEAR;
  int m_d8Far = D8_FAR;
  int m_d8Near = D8_NEAR;

  int m_nDepthColorMapMode = 0;

  DmColorMode(m_ColorPalette, m_nDepthColorMapMode, m_d8Far, m_d8Near);
  DmColorMode11(m_ColorPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
  DmGrayMode11(m_GrayPaletteD11, m_nDepthColorMapMode, m_d11Far, m_d11Near);
  DmColorMode14(m_ColorPaletteZ14, m_zFar, m_zNear);
  DmGrayMode14(m_GrayPaletteZ14, m_zFar, m_zNear);
  // ToDo
}

Image::pointer ColorizerWin::Process(const Image::pointer& depth_buf,
    const DepthMode& depth_mode, bool from_user) {
  // Cache depth buf if from internal
  if (!from_user) CachedDepthBuffer(depth_buf);

  int depth_width = depth_buf->width();
  int depth_height = depth_buf->height();
  if (is_8bits_) {  // 8bits, usb2 (depth width is half, wanted need x2)
    depth_width = depth_width * 2;
  }

  if (depth_mode == DepthMode::DEPTH_RAW) {
    // ImageFormat::DEPTH_RAW
    if (is_8bits_) {  // 8bits, usb2
      auto depth_raw = ImageDepth::Create(ImageFormat::DEPTH_RAW,
          depth_width, depth_height, false);
      depth_raw->set_frame_id(depth_buf->frame_id());
      AdaptU2Raw(depth_buf->data(), depth_raw->data(),
          depth_width, depth_height);
      return depth_raw;
    } else {
      // return clone as it will be changed in imgcallback
      return depth_buf->Clone();
    }
  } else if (depth_mode == DepthMode::DEPTH_COLORFUL) {
    // ImageFormat::DEPTH_RGB
    auto depth_rgb = ImageDepth::Create(ImageFormat::DEPTH_RGB,
        depth_width, depth_height, false);
    depth_rgb->set_frame_id(depth_buf->frame_id());
    UpdateZ14DisplayImage_DIB24(m_ColorPaletteZ14,
        depth_buf->data(), depth_rgb->data(),
        depth_width, depth_height);
    return depth_rgb;
  } else if (depth_mode == DepthMode::DEPTH_GRAY) {
    // ImageFormat::DEPTH_GRAY_24
    auto depth_gray = ImageDepth::Create(ImageFormat::DEPTH_GRAY_24,
        depth_width, depth_height, false);
    depth_gray->set_frame_id(depth_buf->frame_id());
    UpdateZ14DisplayImage_DIB24(m_GrayPaletteZ14,
        depth_buf->data(), depth_gray->data(),
        depth_width, depth_height);
    return depth_gray;
  }
}

MYNTEYE_END_NAMESPACE
