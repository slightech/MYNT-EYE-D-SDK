#ifndef CPROPERTYITEMS_H
#define CPROPERTYITEMS_H

#include <QDialog>
#include <QSpinBox>
#include <QScrollArea>

#include "eSPDI.h"

class MainWindow;
class QGridLayout;
class QRadioButton;
class QCheckBox;
class QSlider;

/*
// for CT PU +
#define PROP_TOTAL_NUMBER 28

#define PROP_TYPE_CT	0
#define PROP_TYPE_PU	1

typedef struct tagPROPERTYITEMINFO {
    char	szPropName[64];
    int		nPropType;
    int		nPropID;
} PROPERTY_ITEM_INFO;

// for CT PU -
*/

class CPropertyItems : public QDialog
{
	Q_OBJECT
	
public:
    explicit CPropertyItems(int nVideoIndex, QWidget *parent = 0);
    ~CPropertyItems();
    
public:
    MainWindow *m_pMainWindow;
    DEVSELINFO m_DevSelInfo;  
    
    QGridLayout *m_pGridLayout;
    
    long int m_nMax;
    long int m_nMin;
    long int m_nStep;
    long int m_nDefault;
    long int m_nCurrentValue;
    long int m_nFlags;
    
    // Camera Terminal+    
    // EXPOSURE MODE
    QRadioButton *m_pAutoModeRadioBtn;
    QRadioButton *m_pManualModeRadioBtn;
    QRadioButton *m_pShutterModeRadioBtn;
    QRadioButton *m_pApertureModeRadioBtn;
    
    // EXPOSURE PRIORITY
    QCheckBox    *m_pAutoExpoPriorityCheckBtn;
    
    // EXPOSURE TIME
    QSlider 	 *m_pAbsExposureTimeSlider;
    QSpinBox     *m_pAbsExposureTimeSpinBox;
    
    QSlider 	 *m_pRelaExposureTimeSlider;
    QSpinBox     *m_pRelaExposureTimeSpinBox;
    
    // FOCUS
    QSlider 	 *m_pAbsFOCUSSlider;
    QSpinBox     *m_pAbsFOCUSSpinBox;
    
    QSlider 	 *m_pRelaFOCUSSlider;
    QSpinBox     *m_pRelaFOCUSSpinBox;
    
    // AUTO FOCUS
    QCheckBox    *m_pAutoFocusCheckBtn;
    
    // IRIS
    QSlider 	 *m_pAbsIRISSlider;
    QSpinBox     *m_pAbsIRISSpinBox;
    
    QSlider 	 *m_pRelaIRISSlider;
    QSpinBox     *m_pRelaIRISSpinBox;
    
    // ROOM
    QSlider 	 *m_pAbsROOMSlider;
    QSpinBox     *m_pAbsROOMSpinBox;
    
    QSlider 	 *m_pRelaROOMSlider;
    QSpinBox     *m_pRelaROOMSpinBox;
    
    // PAN
    QSlider 	 *m_pAbsPANSlider;
    QSpinBox     *m_pAbsPANSpinBox;
    
    QSlider 	 *m_pRelaPANSlider;
    QSpinBox     *m_pRelaPANSpinBox;    
    
    // TILT
    QSlider 	 *m_pAbsTILTSlider;
    QSpinBox     *m_pAbsTILTSpinBox;
    
    QSlider 	 *m_pRelaTILTSlider;
    QSpinBox     *m_pRelaTILTSpinBox; 
    
    // PRIVACY
    QCheckBox    *m_pPrivacyCheckBtn; 
    // Camera Terminal-
    
    // Processing Unit+  
    // Backlight Compensation
    QSlider 	 *m_pBKLCompensationSlider;
    QSpinBox     *m_pBKLCompensationSpinBox; 
    
    // Brightness
    QSlider 	 *m_pBrightnessSlider;
    QSpinBox     *m_pBrightnessSpinBox; 
    
    // Contrast
    QSlider 	 *m_pContrastSlider;
    QSpinBox     *m_pContrastSpinBox;
    
    // Gain
    QSlider 	 *m_pGainSlider;
    QSpinBox     *m_pGainSpinBox;
    
    //
    QRadioButton *m_pPowerLineFreqDisable;
    QRadioButton *m_pPowerLineFreq50Hz;
    QRadioButton *m_pPowerLineFreq60Hz;
    
    // Hue
    QSlider 	 *m_pHueSlider;
    QSpinBox     *m_pHueSpinBox;
    
    // Hue Auto
    QCheckBox    *m_pHueAutoCheckBtn;
    
    // Saturation 
    QSlider 	 *m_pSaturationSlider;
    QSpinBox     *m_pSaturationSpinBox;
    
    // Sharpness 
    QSlider 	 *m_pSharpnessSlider;
    QSpinBox     *m_pSharpnessSpinBox;
    
    // Gamma 
    QSlider 	 *m_pGammaSlider;
    QSpinBox     *m_pGammaSpinBox;
    
    // WB 
    QSlider 	 *m_pWBSlider;
    QSpinBox     *m_pWBSpinBox;
    
    // WB Auto
    QCheckBox    *m_pWBAutoCheckBtn;
    
    // Processing Unit-
    
public slots:

	// Camera Terminal +    	
    // EXPOSURE MODE
	void AutoMode_RadioBtn_Toggled();
	void ManualMode_RadioBtn_Toggled();
	void ShutterMode_RadioBtn_Toggled();
	void ApertureMode_RadioBtn_Toggled();
	
	// EXPOSURE PRIORITY
	void AutoExpoPriority_CheckBtn_Toggled();
	
	// EXPOSURE TIME
	void AbsExpoTime_Slider_ValueChanged(int value);
	void RelaExpoTime_Slider_ValueChanged(int value);
	
	// FOCUS
	void AbsFocus_Slider_ValueChanged(int value);
	void RelaFocus_Slider_ValueChanged(int value);
	
	// AUTO FOCUS
    void AutoFocus_CheckBtn_Toggled();
    
    // IRIS
    void AbsIRIS_Slider_ValueChanged(int value);
	void RelaIRIS_Slider_ValueChanged(int value);
	
	// ROOM
	void AbsRoom_Slider_ValueChanged(int value);
	void RelaRoom_Slider_ValueChanged(int value);
	
	// PAN
	void AbsPan_Slider_ValueChanged(int value);
	void RelaPan_Slider_ValueChanged(int value);
	
	// TILT
	void AbsTilt_Slider_ValueChanged(int value);
	void RelaTilt_Slider_ValueChanged(int value);
	
	// PRIVACY
	void Privacy_CheckBtn_Toggled();
	// Camera Terminal -
	
	// Processing Unit +
	// Backlight Compensation
	void BKLCompensation_Slider_ValueChanged(int value);
	
	// Brightness
	void Brightness_Slider_ValueChanged(int value);
	
	// Contrast
	void Contrast_Slider_ValueChanged(int value);
	
	// Gain
	void Gain_Slider_ValueChanged(int value);
	
	// Power Line Freq.
	void PowerLineFreqDisable_RadioBtn_Toggled();
	void PowerLineFreq50Hz_RadioBtn_Toggled();
	void PowerLineFreq60Hz_RadioBtn_Toggled();
	// Processing Unit -
	
	// Hue
	void Hue_Slider_ValueChanged(int value);
	
	// Hue Auto
	void HueAuto_CheckBtn_Toggled();
	
	// Saturation 
	void Saturation_Slider_ValueChanged(int value);
	
	// Sharpness 
	void Sharpness_Slider_ValueChanged(int value);
	
	// Gamma 
	void Gamma_Slider_ValueChanged(int value);
	
	// WB 
	void WB_Slider_ValueChanged(int value);
	
	// WB Auto
	void WBAuto_CheckBtn_Toggled();
};

#endif // CPROPERTYITEMS_H
