#include "mynteyed/device/device.h"
#include "color_palette_generator.h"

float ColorPaletteGenerator::maxHueAngle = 270.0f;
int ColorPaletteGenerator::defaultAlpha = 255;
void ColorPaletteGenerator::generatePalette(BYTE * palette, int paletteSize, bool reverseRedToBlue) {

	generatePalette(palette, paletteSize, 0, paletteSize);
}

void ColorPaletteGenerator::generatePalette(BYTE * palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseRedToBlue) {
	generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2,0.9f);
}

void ColorPaletteGenerator::generatePalette(BYTE * palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseRedToBlue){

	float R, G, B;
	float step;
	int count = 0;

	if(controlPoint1 > controlPoint2 )controlPoint1 = controlPoint2;

	step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);
	for (int i = 0; i < controlPoint1; i++) {
		getRGB(step * i, R, G, B, reverseRedToBlue);
		palette[count * 4 + 0] = (BYTE)R;
		palette[count * 4 + 1] = (BYTE)G;
		palette[count * 4 + 2] = (BYTE)B;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	step = (controlPoint2Value - controlPoint1Value) / (float)(controlPoint2 - controlPoint1);
	for (int i = 0; i< controlPoint2 - controlPoint1; i++) {
		getRGB(step * i + controlPoint1Value, R, G, B, reverseRedToBlue);
		palette[count * 4 + 0] = (BYTE)R;
		palette[count * 4 + 1] = (BYTE)G;
		palette[count * 4 + 2] = (BYTE)B;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	step = (1.0f - controlPoint2Value) / (float)(paletteSize - controlPoint2);
	for (int i = 0; i < paletteSize - controlPoint2; i++) {
		getRGB(step * i + controlPoint2Value, R, G, B, reverseRedToBlue);
		palette[count * 4 + 0] = (BYTE)R;
		palette[count * 4 + 1] = (BYTE)G;
		palette[count * 4 + 2] = (BYTE)B;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	palette[0] = 0;
	palette[1] = 0;
	palette[2] = 0;
	palette[3] = (BYTE)defaultAlpha;
	
	//palette[(paletteSize - 1)*4 + 0] = 255;
	//palette[(paletteSize - 1)*4 + 1] = 255;
	//palette[(paletteSize - 1)*4 + 2] = 255;
	//palette[(paletteSize - 1)*4 + 3] = (BYTE)defaultAlpha;
}
void ColorPaletteGenerator::generatePaletteGray(BYTE * palette, int paletteSize, bool reverseGraylevel) {

	generatePalette(palette, paletteSize, 0, paletteSize);
}

void ColorPaletteGenerator::generatePaletteGray(BYTE * palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseGraylevel) {
	generatePalette(palette, paletteSize, controlPoint1, 0.1f, controlPoint2, 0.9f);
}

void ColorPaletteGenerator::generatePaletteGray(BYTE * palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseGraylevel) {
	if (controlPoint1 > controlPoint2)controlPoint1 = controlPoint2;
	float step;
	step = (controlPoint1Value - 0.0f) / (float)(controlPoint1 - 0);

	int count = 0;
	for (int i = 0; i < controlPoint1; i++) {
		int grayValue = 255 * (reverseGraylevel ? 1.0f - (step*i) : (step*i));
		palette[count * 4 + 0] = (BYTE)grayValue;
		palette[count * 4 + 1] = (BYTE)grayValue;
		palette[count * 4 + 2] = (BYTE)grayValue;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	step = (controlPoint2Value - controlPoint1Value) / (float)(controlPoint2 - controlPoint1);
	for (int i = 0; i< controlPoint2 - controlPoint1; i++) {		
		int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint1Value)) : ((step * i + controlPoint1Value)));
		palette[count * 4 + 0] = (BYTE)grayValue;
		palette[count * 4 + 1] = (BYTE)grayValue;
		palette[count * 4 + 2] = (BYTE)grayValue;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	step = (1.0f - controlPoint2Value) / (float)(paletteSize - controlPoint2);
	for (int i = 0; i < paletteSize - controlPoint2; i++) {
		int grayValue = 255 * (reverseGraylevel ? 1.0f - ((step * i + controlPoint2Value)) : ((step * i + controlPoint2Value)));
		palette[count * 4 + 0] = (BYTE)grayValue;
		palette[count * 4 + 1] = (BYTE)grayValue;
		palette[count * 4 + 2] = (BYTE)grayValue;
		palette[count * 4 + 3] = (BYTE)defaultAlpha;
		count++;
	}

	palette[0] = 0;
	palette[1] = 0;
	palette[2] = 0;
	palette[3] = (BYTE)defaultAlpha;
	//palette[(paletteSize - 1)*4 + 0] = 255;
	//palette[(paletteSize - 1)*4 + 1] = 255;
	//palette[(paletteSize - 1)*4 + 2] = 255;
	//palette[(paletteSize - 1)*4 + 3] = (BYTE)defaultAlpha;
}
void ColorPaletteGenerator::getRGB(float value, float &R, float &G, float &B, bool reverseRedToBlue ) {	

	if (reverseRedToBlue) value = 1.0f - value;
	HSV_to_RGB(value * maxHueAngle, 1.0f,1.0f,R,G,B);
}

void ColorPaletteGenerator::HSV_to_RGB(float H, float S, float V, float &R, float &G, float &B){
	float nMax, nMin;
	float fDet;
	//
	while (H<0.0) H += 360.0;
	while (H >= 360.0) H -= 360.0;
	H /= 60.0;
	if (V<0.0) V = 0.0;
	if (V>1.0) V = 1.0;
	V *= 255.0;
	if (S<0.0) S = 0.0;
	if (S>1.0) S = 1.0;
	//
	if (V == 0.0) {
		R = G = B = 0;
	}
	else {
		fDet = S * V;
		nMax = (V);
		nMin = (V - fDet);
		if (H <= 1.0) { //R>=G>=B, H=(G-B)/fDet
			R = nMax;
			B = nMin;
			G = (H*fDet + B);
		}
		else if (H <= 2.0) { //G>=R>=B, H=2+(B-R)/fDet
			G = nMax;
			B = nMin;
			R = ((2.0 - H)*fDet + B);
		}
		else if (H <= 3.0) { //G>=B>=R, H=2+(B-R)/fDet
			G = nMax;
			R = nMin;
			B = ((H - 2.0)*fDet + R);
		}
		else if (H <= 4.0) { //B>=G>=R, H=4+(R-G)/fDet
			B = nMax;
			R = nMin;
			G = ((4.0 - H)*fDet + R);
		}
		else if (H <= 5.0) { //B>=R>=G, H=4+(R-G)/fDet
			B = nMax;
			G = nMin;
			R = ((H - 4.0)*fDet + G);
		}
		else { // if(H<6.0) //R>=B>=G, H=(G-B)/fDet+6
			R = nMax;
			G = nMin;
			B = ((6.0 - H)*fDet + G);
		}
	}
}