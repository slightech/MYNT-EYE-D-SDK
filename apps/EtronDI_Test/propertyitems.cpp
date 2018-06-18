// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "propertyitems.h"
#include "videodevicedlg.h"
#include "mainwindow.h"

CPropertyItems::CPropertyItems(int nVideoIndex, QWidget *parent) :
                         QDialog(parent)
{
	CVideoDeviceDlg *pDlg = (CVideoDeviceDlg*)parent;
	
	m_pMainWindow = pDlg->m_pMainWindow;
    m_DevSelInfo.index = nVideoIndex;
    
    m_nMax = 0;
    m_nMin = 0;
    m_nStep = 0;
    m_nDefault = 0;
    m_nFlags = 0;
    m_nCurrentValue = 0;
    
    m_pGridLayout = new QGridLayout();
    
    QFont font = QFont("Courier", 10, QFont::Bold);
    setFont(font);
    
    /*
    // Camera Terminal +
    // EXPOSURE MODE
    if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QGroupBox *box = new QGroupBox("Auto Exposure Mode Control:");
		m_pAutoModeRadioBtn     = new QRadioButton("Auto Mode");
		m_pManualModeRadioBtn   = new QRadioButton("Manual Mode");
		m_pShutterModeRadioBtn  = new QRadioButton("Shutter Priority Mode");
		m_pApertureModeRadioBtn = new QRadioButton("Aperture Priority Mode");
		
		if( ETronDI_OK != EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &m_nCurrentValue)) m_nCurrentValue = m_nDefault;
		
		switch(m_nCurrentValue) {

			case 0:
				m_pAutoModeRadioBtn->setChecked(true);
				break;
			case 1:
				m_pManualModeRadioBtn->setChecked(true);
				break;
			case 2:
				m_pShutterModeRadioBtn->setChecked(true);
				break;
			case 3:
				m_pApertureModeRadioBtn->setChecked(true);
				break;
		}
		
		QVBoxLayout *radioLayout = new QVBoxLayout;		
		radioLayout->addWidget(m_pAutoModeRadioBtn);
		radioLayout->addWidget(m_pManualModeRadioBtn);
		radioLayout->addWidget(m_pShutterModeRadioBtn);
		radioLayout->addWidget(m_pApertureModeRadioBtn);
		box->setLayout(radioLayout);
		
        m_pGridLayout->addWidget(box, 0, 0, 1, 5);
		
		connect( m_pAutoModeRadioBtn,     SIGNAL(toggled(bool)), this, SLOT(AutoMode_RadioBtn_Toggled()));
		connect( m_pManualModeRadioBtn,   SIGNAL(toggled(bool)), this, SLOT(ManualMode_RadioBtn_Toggled()));
		connect( m_pShutterModeRadioBtn,  SIGNAL(toggled(bool)), this, SLOT(ShutterMode_RadioBtn_Toggled()));
		connect( m_pApertureModeRadioBtn, SIGNAL(toggled(bool)), this, SLOT(ApertureMode_RadioBtn_Toggled()));
	}
	
	// EXPOSURE PRIORITY
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Auto Exposure Priority Control:");
		m_pAutoExpoPriorityCheckBtn = new QCheckBox;
		//m_pAutoExpoPriorityCheckBtn->setText("Auto Exposure Priority Control");
		
		if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL, &m_nCurrentValue) ) {
			if(m_nCurrentValue == 0) m_pAutoExpoPriorityCheckBtn->setChecked(false);
			else m_pAutoExpoPriorityCheckBtn->setChecked(true);
		}
		
		m_pGridLayout->addWidget(label, 1, 0, 1, 1);
		m_pGridLayout->addWidget(m_pAutoExpoPriorityCheckBtn, 1, 1, 1, 1);
		
		connect( m_pAutoExpoPriorityCheckBtn, SIGNAL(toggled(bool)), this, SLOT(AutoExpoPriority_CheckBtn_Toggled()));
	}
	
	// EXPOSURE TIME 
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
        QLabel *label = new QLabel("Absolute Expo Time:");
        QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);

		m_pAbsExposureTimeSpinBox = new QSpinBox; 
		m_pAbsExposureTimeSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsExposureTimeSpinBox->setSingleStep(m_nStep);
		
		m_pAbsExposureTimeSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsExposureTimeSlider->setRange(m_nMin, m_nMax);
		m_pAbsExposureTimeSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pAbsExposureTimeSlider->setValue(m_nCurrentValue);
            m_pAbsExposureTimeSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsExposureTimeSlider,  SIGNAL(valueChanged(int)), m_pAbsExposureTimeSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsExposureTimeSpinBox, SIGNAL(valueChanged(int)), m_pAbsExposureTimeSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 2, 0, 1, 1);
        m_pGridLayout->addWidget(min, 2, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsExposureTimeSlider, 2, 2, 1, 1);
        m_pGridLayout->addWidget(max, 2, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsExposureTimeSpinBox, 2, 4, 1, 1);
        
		connect( m_pAbsExposureTimeSlider,  SIGNAL(valueChanged(int)), this, SLOT(AbsExpoTime_Slider_ValueChanged(int))); 
	}
		
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Relative Exposure Time:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaExposureTimeSpinBox = new QSpinBox; 
		m_pRelaExposureTimeSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaExposureTimeSpinBox->setSingleStep(m_nStep);
		
		m_pRelaExposureTimeSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaExposureTimeSlider->setRange(m_nMin, m_nMax);
		m_pRelaExposureTimeSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL, &m_nCurrentValue) ) {
            m_pRelaExposureTimeSlider->setValue(m_nCurrentValue);
            m_pRelaExposureTimeSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaExposureTimeSlider,  SIGNAL(valueChanged(int)), m_pRelaExposureTimeSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaExposureTimeSpinBox, SIGNAL(valueChanged(int)), m_pRelaExposureTimeSlider,  SLOT(setValue(int))); 
         
        m_pGridLayout->addWidget(label, 3, 0, 1, 1);
        m_pGridLayout->addWidget(min, 3, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaExposureTimeSlider, 3, 2, 1, 1);
        m_pGridLayout->addWidget(max, 3, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaExposureTimeSpinBox, 3, 4, 1, 1);
        
        connect( m_pRelaExposureTimeSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaExpoTime_Slider_ValueChanged(int)));
	}
	
	// Focus
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Absolute Focus:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pAbsFOCUSSpinBox = new QSpinBox; 
		m_pAbsFOCUSSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsFOCUSSpinBox->setSingleStep(m_nStep);
		
		m_pAbsFOCUSSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsFOCUSSlider->setRange(m_nMin, m_nMax);
		m_pAbsFOCUSSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pAbsFOCUSSlider->setValue(m_nCurrentValue);
            m_pAbsFOCUSSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsFOCUSSlider,  SIGNAL(valueChanged(int)), m_pAbsFOCUSSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsFOCUSSpinBox, SIGNAL(valueChanged(int)), m_pAbsFOCUSSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 4, 0, 1, 1);
        m_pGridLayout->addWidget(min, 4, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsFOCUSSlider, 4, 2, 1, 1);
        m_pGridLayout->addWidget(max, 4, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsFOCUSSpinBox, 4, 4, 1, 1);
        
        connect( m_pAbsFOCUSSlider, SIGNAL(valueChanged(int)), this, SLOT(AbsFocus_Slider_ValueChanged(int))); 
	}
	
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Relative Focus:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaFOCUSSpinBox = new QSpinBox; 
		m_pRelaFOCUSSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaFOCUSSpinBox->setSingleStep(m_nStep);
		
		m_pRelaFOCUSSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaFOCUSSlider->setRange(m_nMin, m_nMax);
		m_pRelaFOCUSSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pRelaFOCUSSlider->setValue(m_nCurrentValue);
            m_pRelaFOCUSSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaFOCUSSlider,  SIGNAL(valueChanged(int)), m_pRelaFOCUSSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaFOCUSSpinBox, SIGNAL(valueChanged(int)), m_pRelaFOCUSSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 5, 0, 1, 1);
        m_pGridLayout->addWidget(min, 5, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaFOCUSSlider, 5, 2, 1, 3);
        m_pGridLayout->addWidget(max, 5, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaFOCUSSpinBox, 5, 4, 1, 1);
         
        connect( m_pRelaFOCUSSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaFocus_Slider_ValueChanged(int))); 
	}
	
	// FOCUS Auto
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_AUTO_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Auto Focus Control:");
		m_pAutoFocusCheckBtn = new QCheckBox;
		//m_pAutoFocusCheckBtn->setText("Auto Focus Control");
		
		if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_AUTO_CTRL, &m_nCurrentValue) ) {
			if(m_nCurrentValue == 0) m_pAutoFocusCheckBtn->setChecked(false);
			else m_pAutoFocusCheckBtn->setChecked(true);
		}
		
		m_pGridLayout->addWidget(label, 6, 0, 1, 1);
		m_pGridLayout->addWidget(m_pAutoFocusCheckBtn, 6, 1, 1, 1);
		
		connect( m_pAutoFocusCheckBtn, SIGNAL(toggled(bool)), this, SLOT(AutoFocus_CheckBtn_Toggled()));
	}
		
	// Iris
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Absolute Iris:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pAbsIRISSpinBox = new QSpinBox; 
		m_pAbsIRISSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsIRISSpinBox->setSingleStep(m_nStep);
		
		m_pAbsIRISSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsIRISSlider->setRange(m_nMin, m_nMax);
		m_pAbsIRISSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pAbsIRISSlider->setValue(m_nCurrentValue);
            m_pAbsIRISSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsIRISSlider,  SIGNAL(valueChanged(int)), m_pAbsIRISSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsIRISSpinBox, SIGNAL(valueChanged(int)), m_pAbsIRISSlider,  SLOT(setValue(int))); 
         
        m_pGridLayout->addWidget(label, 7, 0, 1, 1);
        m_pGridLayout->addWidget(min, 7, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsIRISSlider, 7, 2, 1, 1);
        m_pGridLayout->addWidget(max, 7, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsIRISSpinBox, 7, 4, 1, 1);
        
        connect( m_pAbsIRISSlider, SIGNAL(valueChanged(int)), this, SLOT(AbsIRIS_Slider_ValueChanged(int))); 
	}
	
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Relative Iris:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaIRISSpinBox = new QSpinBox; 
		m_pRelaIRISSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaIRISSpinBox->setSingleStep(m_nStep);
		
		m_pRelaIRISSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaIRISSlider->setRange(m_nMin, m_nMax);
		m_pRelaIRISSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_RELATIVE_CTRL, &m_nCurrentValue) ) {
            m_pRelaIRISSlider->setValue(m_nCurrentValue);
            m_pRelaIRISSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaIRISSlider,  SIGNAL(valueChanged(int)), m_pRelaIRISSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaIRISSpinBox, SIGNAL(valueChanged(int)), m_pRelaIRISSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 8, 0, 1, 1);
        m_pGridLayout->addWidget(min, 8, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaIRISSlider, 8, 2, 1, 1);
        m_pGridLayout->addWidget(max, 8, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaIRISSpinBox, 8, 4, 1, 1);
        
        connect( m_pRelaIRISSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaIRIS_Slider_ValueChanged(int))); 
	}
	
	// ROOM
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Absolute Room:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pAbsROOMSpinBox = new QSpinBox; 
		m_pAbsROOMSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsROOMSpinBox->setSingleStep(m_nStep);
		
		m_pAbsROOMSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsROOMSlider->setRange(m_nMin, m_nMax);
		m_pAbsROOMSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pRelaIRISSlider->setValue(m_nCurrentValue);
            m_pAbsROOMSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsROOMSlider,  SIGNAL(valueChanged(int)), m_pAbsROOMSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsROOMSpinBox, SIGNAL(valueChanged(int)), m_pAbsROOMSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 9, 0, 1, 1);
        m_pGridLayout->addWidget(min, 9, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsROOMSlider, 9, 2, 1, 1);
        m_pGridLayout->addWidget(max, 9, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsROOMSpinBox, 9, 4, 1, 1);	
        
        connect( m_pAbsROOMSlider, SIGNAL(valueChanged(int)), this, SLOT(AbsRoom_Slider_ValueChanged(int)));	
	}
	
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Relative Room:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaROOMSpinBox = new QSpinBox; 
		m_pRelaROOMSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaROOMSpinBox->setSingleStep(m_nStep);
		
		m_pRelaROOMSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaROOMSlider->setRange(m_nMin, m_nMax);
		m_pRelaROOMSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL, &m_nCurrentValue) ) {
            m_pRelaIRISSlider->setValue(m_nCurrentValue);
            m_pRelaROOMSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaROOMSlider,  SIGNAL(valueChanged(int)), m_pRelaROOMSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaROOMSpinBox, SIGNAL(valueChanged(int)), m_pRelaROOMSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 10, 0, 1, 1);
        m_pGridLayout->addWidget(min, 10, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaROOMSlider, 10, 2, 1, 1);
        m_pGridLayout->addWidget(max, 10, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaROOMSpinBox, 10, 4, 1, 1);	
        
        connect( m_pRelaROOMSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaRoom_Slider_ValueChanged(int))); 
	}
	
	// PAN
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Absolute PAN:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pAbsPANSpinBox = new QSpinBox; 
		m_pAbsPANSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsPANSpinBox->setSingleStep(m_nStep);
		
		m_pAbsPANSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsPANSlider->setRange(m_nMin, m_nMax);
		m_pAbsPANSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pRelaIRISSlider->setValue(m_nCurrentValue);
            m_pAbsROOMSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsPANSlider,  SIGNAL(valueChanged(int)), m_pAbsPANSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsPANSpinBox, SIGNAL(valueChanged(int)), m_pAbsPANSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 11, 0, 1, 1);
        m_pGridLayout->addWidget(min, 11, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsPANSlider, 11, 2, 1, 1);
        m_pGridLayout->addWidget(max, 11, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsPANSpinBox, 11, 4, 1, 1);        
        
        connect( m_pAbsPANSlider, SIGNAL(valueChanged(int)), this, SLOT(AbsPan_Slider_ValueChanged(int))); 	
	}
	
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Relative PAN:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaPANSpinBox = new QSpinBox; 
		m_pRelaPANSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaPANSpinBox->setSingleStep(m_nStep);
		
		m_pRelaPANSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaPANSlider->setRange(m_nMin, m_nMax);
		m_pRelaPANSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_RELATIVE_CTRL, &m_nCurrentValue) ) {
            m_pRelaPANSlider->setValue(m_nCurrentValue);
            m_pRelaPANSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaPANSlider,  SIGNAL(valueChanged(int)), m_pRelaPANSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaPANSpinBox, SIGNAL(valueChanged(int)), m_pRelaPANSlider,  SLOT(setValue(int))); 
        
		m_pGridLayout->addWidget(label, 12, 0, 1, 1);
        m_pGridLayout->addWidget(min, 12, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaPANSlider, 12, 2, 1, 1);
        m_pGridLayout->addWidget(max, 12, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaPANSpinBox, 12, 4, 1, 1); 
        
        connect( m_pRelaPANSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaPan_Slider_ValueChanged(int))); 		
	}
	
	// TILT
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Absolute TILE:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pAbsTILTSpinBox = new QSpinBox; 
		m_pAbsTILTSpinBox->setRange(m_nMin, m_nMax); 
		m_pAbsTILTSpinBox->setSingleStep(m_nStep);
		
		m_pAbsTILTSlider = new QSlider(Qt::Horizontal, parent);
		m_pAbsTILTSlider->setRange(m_nMin, m_nMax);
		m_pAbsTILTSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL, &m_nCurrentValue) ) {
            m_pAbsTILTSlider->setValue(m_nCurrentValue);
            m_pAbsTILTSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pAbsTILTSlider,  SIGNAL(valueChanged(int)), m_pAbsTILTSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pAbsTILTSpinBox, SIGNAL(valueChanged(int)), m_pAbsTILTSlider,  SLOT(setValue(int))); 	
        
        m_pGridLayout->addWidget(label, 13, 0, 1, 1);
        m_pGridLayout->addWidget(min, 13, 1, 1, 1);
        m_pGridLayout->addWidget(m_pAbsTILTSlider, 13, 2, 1, 1);
        m_pGridLayout->addWidget(max, 13, 3, 1, 1);
        m_pGridLayout->addWidget(m_pAbsTILTSpinBox, 13, 4, 1, 1); 
        
        connect( m_pAbsTILTSlider, SIGNAL(valueChanged(int)), this, SLOT(AbsTilt_Slider_ValueChanged(int)));  	
	}
	
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_RELATIVE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {

		QLabel *label = new QLabel("Relative TILE:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pRelaTILTSpinBox = new QSpinBox; 
		m_pRelaTILTSpinBox->setRange(m_nMin, m_nMax); 
		m_pRelaTILTSpinBox->setSingleStep(m_nStep);
		
		m_pRelaTILTSlider = new QSlider(Qt::Horizontal, parent);
		m_pRelaTILTSlider->setRange(m_nMin, m_nMax);
		m_pRelaTILTSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_RELATIVE_CTRL, &m_nCurrentValue) ) {
            m_pRelaTILTSlider->setValue(m_nCurrentValue);
            m_pRelaTILTSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pRelaTILTSlider,  SIGNAL(valueChanged(int)), m_pRelaTILTSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pRelaTILTSpinBox, SIGNAL(valueChanged(int)), m_pRelaTILTSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 14, 0, 1, 1);
        m_pGridLayout->addWidget(min, 14, 1, 1, 1);
        m_pGridLayout->addWidget(m_pRelaTILTSlider, 14, 2, 1, 1);
        m_pGridLayout->addWidget(max, 14, 3, 1, 1);
        m_pGridLayout->addWidget(m_pRelaTILTSpinBox, 14, 4, 1, 1); 
        
        connect( m_pRelaTILTSlider, SIGNAL(valueChanged(int)), this, SLOT(RelaTilt_Slider_ValueChanged(int)));	
	}
	
	// Privacy
	if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PRIVACY_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Privacy Control:");
		m_pPrivacyCheckBtn = new QCheckBox;
		//m_pPrivacyCheckBtn->setText("Privacy Control");
		
		if( ETronDI_OK == EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PRIVACY_CTRL, &m_nCurrentValue) ) {
			if(m_nCurrentValue == 0) m_pPrivacyCheckBtn->setChecked(false);
			else m_pPrivacyCheckBtn->setChecked(true);
		}
		
		m_pGridLayout->addWidget(label, 15, 0, 1, 1);
		m_pGridLayout->addWidget(m_pAutoFocusCheckBtn, 15, 1, 1, 1);
		
		connect( m_pPrivacyCheckBtn, SIGNAL(toggled(bool)), this, SLOT(Privacy_CheckBtn_Toggled())); 
	}
	
	// Camera Terminal -
	
	// Processing Unit +
	// Backlight Compensation
	if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Backlight Compensation:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pBKLCompensationSpinBox = new QSpinBox; 
		m_pBKLCompensationSpinBox->setRange(m_nMin, m_nMax); 
		m_pBKLCompensationSpinBox->setSingleStep(m_nStep);
		
		m_pBKLCompensationSlider = new QSlider(Qt::Horizontal, parent);
		m_pBKLCompensationSlider->setRange(m_nMin, m_nMax);
		m_pBKLCompensationSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL, &m_nCurrentValue) ) {
            m_pBKLCompensationSlider->setValue(m_nCurrentValue);
            m_pBKLCompensationSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pBKLCompensationSlider,  SIGNAL(valueChanged(int)), m_pBKLCompensationSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pBKLCompensationSpinBox, SIGNAL(valueChanged(int)), m_pBKLCompensationSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 16, 0, 1, 1);
        m_pGridLayout->addWidget(min, 16, 1, 1, 1);
        m_pGridLayout->addWidget(m_pBKLCompensationSlider, 16, 2, 1, 1);
        m_pGridLayout->addWidget(max, 16, 3, 1, 1);
        m_pGridLayout->addWidget(m_pBKLCompensationSpinBox, 16, 4, 1, 1);
        
        connect( m_pBKLCompensationSlider, SIGNAL(valueChanged(int)), this, SLOT(BKLCompensation_Slider_ValueChanged(int)));
	}

	// Brightness
	if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BRIGHTNESS_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Brightness:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pBrightnessSpinBox = new QSpinBox; 
		m_pBrightnessSpinBox->setRange(m_nMin, m_nMax); 
		m_pBrightnessSpinBox->setSingleStep(m_nStep);
		
		m_pBrightnessSlider = new QSlider(Qt::Horizontal, parent);
		m_pBrightnessSlider->setRange(m_nMin, m_nMax);
		m_pBrightnessSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BRIGHTNESS_CTRL, &m_nCurrentValue) ) {
            m_pBrightnessSlider->setValue(m_nCurrentValue);
            m_pBrightnessSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pBrightnessSlider,  SIGNAL(valueChanged(int)), m_pBrightnessSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pBrightnessSpinBox, SIGNAL(valueChanged(int)), m_pBrightnessSlider,  SLOT(setValue(int))); 

        m_pGridLayout->addWidget(label, 17, 0, 1, 1);
        m_pGridLayout->addWidget(min, 17, 1, 1, 1);
        m_pGridLayout->addWidget(m_pBrightnessSlider, 17, 2, 1, 1);
        m_pGridLayout->addWidget(max, 17, 3, 1, 1);
        m_pGridLayout->addWidget(m_pBrightnessSpinBox, 17, 4, 1, 1);
        
        connect( m_pBrightnessSlider, SIGNAL(valueChanged(int)), this, SLOT(Brightness_Slider_ValueChanged(int)));
	}

	// Contrast
	if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_CONTRAST_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Contrast:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));        
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pContrastSpinBox = new QSpinBox; 
		m_pContrastSpinBox->setRange(m_nMin, m_nMax); 
		m_pContrastSpinBox->setSingleStep(m_nStep);
		
		m_pContrastSlider = new QSlider(Qt::Horizontal, parent);
		m_pContrastSlider->setRange(m_nMin, m_nMax);
		m_pContrastSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_CONTRAST_CTRL, &m_nCurrentValue) ) {
            m_pContrastSlider->setValue(m_nCurrentValue);
            m_pContrastSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pContrastSlider,  SIGNAL(valueChanged(int)), m_pContrastSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pContrastSpinBox, SIGNAL(valueChanged(int)), m_pContrastSlider,  SLOT(setValue(int))); 	
        
        m_pGridLayout->addWidget(label, 18, 0, 1, 1);
        m_pGridLayout->addWidget(min, 18, 1, 1, 1);
        m_pGridLayout->addWidget(m_pContrastSlider, 18, 2, 1, 1);
        m_pGridLayout->addWidget(max, 18, 3, 1, 1);
        m_pGridLayout->addWidget(m_pContrastSpinBox, 18, 4, 1, 1);
        
        connect( m_pContrastSlider, SIGNAL(valueChanged(int)), this, SLOT(Contrast_Slider_ValueChanged(int)));
	}
	
	//Gain
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAIN_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Gain:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pGainSpinBox = new QSpinBox; 
		m_pGainSpinBox->setRange(m_nMin, m_nMax); 
		m_pGainSpinBox->setSingleStep(m_nStep);
		
		m_pGainSlider = new QSlider(Qt::Horizontal, parent);
		m_pGainSlider->setRange(m_nMin, m_nMax);
		m_pGainSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAIN_CTRL, &m_nCurrentValue) ) {
            m_pGainSlider->setValue(m_nCurrentValue);
            m_pGainSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pGainSlider,  SIGNAL(valueChanged(int)), m_pGainSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pGainSpinBox, SIGNAL(valueChanged(int)), m_pGainSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 19, 0, 1, 1);
        m_pGridLayout->addWidget(min, 19, 1, 1, 1);
        m_pGridLayout->addWidget(m_pBrightnessSlider, 19, 2, 1, 1);
        m_pGridLayout->addWidget(max, 19, 3, 1, 1);
        m_pGridLayout->addWidget(m_pBrightnessSpinBox, 19, 4, 1, 1);	
        
        connect( m_pGainSlider, SIGNAL(valueChanged(int)), this, SLOT(Gain_Slider_ValueChanged(int))); 
    }
	
	// Power Line Freq.
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QGroupBox *box = new QGroupBox("Power Line Frequency Control:");
		m_pPowerLineFreqDisable = new QRadioButton("Disable");
		m_pPowerLineFreq50Hz    = new QRadioButton("50 Hz");
		m_pPowerLineFreq60Hz    = new QRadioButton("60 Hz");
		
		if( ETronDI_OK != EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, &m_nCurrentValue)) m_nCurrentValue = m_nDefault;
		
		switch(m_nCurrentValue) {

			case 0:
				m_pPowerLineFreqDisable->setChecked(true);
				break;
			case 1:
				m_pPowerLineFreq50Hz->setChecked(true);
				break;
			case 2:
				m_pPowerLineFreq60Hz->setChecked(true);
				break;
		}
		
		QVBoxLayout *radioLayout = new QVBoxLayout;		
		radioLayout->addWidget(m_pPowerLineFreqDisable);
		radioLayout->addWidget(m_pPowerLineFreq50Hz);
		radioLayout->addWidget(m_pPowerLineFreq60Hz);
		box->setLayout(radioLayout);
		
		m_pGridLayout->addWidget(box, 20, 0, 1, 5);
		
		connect( m_pPowerLineFreqDisable, SIGNAL(toggled(bool)), this, SLOT(PowerLineFreqDisable_RadioBtn_Toggled()));
		connect( m_pPowerLineFreq50Hz,    SIGNAL(toggled(bool)), this, SLOT(PowerLineFreq50Hz_RadioBtn_Toggled()));
		connect( m_pPowerLineFreq60Hz,    SIGNAL(toggled(bool)), this, SLOT(PowerLineFreq60Hz_RadioBtn_Toggled()));
    }
	
	// Hue
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Hue:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pHueSpinBox = new QSpinBox; 
		m_pHueSpinBox->setRange(m_nMin, m_nMax); 
		m_pHueSpinBox->setSingleStep(m_nStep);
		
		m_pHueSlider = new QSlider(Qt::Horizontal, parent);
		m_pHueSlider->setRange(m_nMin, m_nMax);
		m_pHueSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_CTRL, &m_nCurrentValue) ) {
            m_pHueSlider->setValue(m_nCurrentValue);
            m_pHueSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pHueSlider,  SIGNAL(valueChanged(int)), m_pHueSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pHueSpinBox, SIGNAL(valueChanged(int)), m_pHueSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 21, 0, 1, 1);
        m_pGridLayout->addWidget(min, 21, 1, 1, 1);
        m_pGridLayout->addWidget(m_pHueSlider, 21, 2, 1, 1);
        m_pGridLayout->addWidget(max, 21, 3, 1, 1);
        m_pGridLayout->addWidget(m_pHueSpinBox, 21, 4, 1, 1);		
        
        connect( m_pHueSlider, SIGNAL(valueChanged(int)), this, SLOT(Hue_Slider_ValueChanged(int)));
    }
	
	// Hue Auto
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_AUTO_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Hue Auto Control:");
		m_pHueAutoCheckBtn = new QCheckBox;
		//m_pHueAutoCheckBtn->setText("Hue Auto Control");
		
		if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_AUTO_CTRL, &m_nCurrentValue) ) {
			if(m_nCurrentValue == 0) m_pHueAutoCheckBtn->setChecked(false);
			else m_pHueAutoCheckBtn->setChecked(true);
		}
		
		m_pGridLayout->addWidget(label, 22, 0, 1, 1);
		m_pGridLayout->addWidget(m_pHueAutoCheckBtn, 22, 1, 1, 1);
		
		connect( m_pHueAutoCheckBtn, SIGNAL(toggled(bool)), this, SLOT(HueAuto_CheckBtn_Toggled()));
    }
	
	// Saturation 
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SATURATION_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Saturation:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pSaturationSpinBox = new QSpinBox; 
		m_pSaturationSpinBox->setRange(m_nMin, m_nMax); 
		m_pSaturationSpinBox->setSingleStep(m_nStep);
		
		m_pSaturationSlider = new QSlider(Qt::Horizontal, parent);
		m_pSaturationSlider->setRange(m_nMin, m_nMax);
		m_pSaturationSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SATURATION_CTRL, &m_nCurrentValue) ) {
            m_pSaturationSlider->setValue(m_nCurrentValue);
            m_pSaturationSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pSaturationSlider,  SIGNAL(valueChanged(int)), m_pSaturationSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pSaturationSpinBox, SIGNAL(valueChanged(int)), m_pSaturationSlider,  SLOT(setValue(int))); 	
        
        m_pGridLayout->addWidget(label, 23, 0, 1, 1);
        m_pGridLayout->addWidget(min, 23, 1, 1, 1);
        m_pGridLayout->addWidget(m_pSaturationSlider, 23, 2, 1, 1);
        m_pGridLayout->addWidget(max, 23, 3, 1, 1);
        m_pGridLayout->addWidget(m_pSaturationSpinBox, 23, 4, 1, 1);	
        
        connect( m_pSaturationSlider, SIGNAL(valueChanged(int)), this, SLOT(Saturation_Slider_ValueChanged(int)));
    }
	
	// Sharpness
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SHARPNESS_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Sharpness:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pSharpnessSpinBox = new QSpinBox; 
		m_pSharpnessSpinBox->setRange(m_nMin, m_nMax); 
		m_pSharpnessSpinBox->setSingleStep(m_nStep);
		
		m_pSharpnessSlider = new QSlider(Qt::Horizontal, parent);
		m_pSharpnessSlider->setRange(m_nMin, m_nMax);
		m_pSharpnessSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SHARPNESS_CTRL, &m_nCurrentValue) ) {
            m_pSharpnessSlider->setValue(m_nCurrentValue);
            m_pSharpnessSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pSharpnessSlider,  SIGNAL(valueChanged(int)), m_pSharpnessSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pSharpnessSpinBox, SIGNAL(valueChanged(int)), m_pSharpnessSlider,  SLOT(setValue(int))); 	
        
        m_pGridLayout->addWidget(label, 24, 0, 1, 1);
        m_pGridLayout->addWidget(min, 24, 1, 1, 1);
        m_pGridLayout->addWidget(m_pSharpnessSlider, 24, 2, 1, 1);
        m_pGridLayout->addWidget(max, 24, 3, 1, 1);
        m_pGridLayout->addWidget(m_pSharpnessSpinBox, 24, 4, 1, 1);
        
        connect( m_pSharpnessSlider, SIGNAL(valueChanged(int)), this, SLOT(Sharpness_Slider_ValueChanged(int)));
    }
	
	// Gamma
	if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAMMA_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("Gamma:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pGammaSpinBox = new QSpinBox; 
		m_pGammaSpinBox->setRange(m_nMin, m_nMax); 
		m_pGammaSpinBox->setSingleStep(m_nStep);
		
		m_pGammaSlider = new QSlider(Qt::Horizontal, parent);
		m_pGammaSlider->setRange(m_nMin, m_nMax);
		m_pGammaSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAMMA_CTRL, &m_nCurrentValue) ) {
            m_pGammaSlider->setValue(m_nCurrentValue);
            m_pGammaSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pGammaSlider,  SIGNAL(valueChanged(int)), m_pGammaSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pGammaSpinBox, SIGNAL(valueChanged(int)), m_pGammaSlider,  SLOT(setValue(int))); 
        
        m_pGridLayout->addWidget(label, 25, 0, 1, 1);
        m_pGridLayout->addWidget(min, 25, 1, 1, 1);
        m_pGridLayout->addWidget(m_pGammaSlider, 25, 2, 1, 1);
        m_pGridLayout->addWidget(max, 25, 3, 1, 1);
        m_pGridLayout->addWidget(m_pGammaSpinBox, 25, 4, 1, 1);	
        
        connect( m_pGammaSlider, SIGNAL(valueChanged(int)), this, SLOT(Gamma_Slider_ValueChanged(int)));
	}
	
	// WB
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("White Balance:");
		QLabel *min = new QLabel;
        min->setText(QString::number(m_nMin));
        QLabel *max = new QLabel;
        max->setText(QString::number(m_nMax));
        
        min->setAlignment(Qt::AlignCenter);
        max->setAlignment(Qt::AlignCenter);
		
		m_pWBSpinBox = new QSpinBox; 
		m_pWBSpinBox->setRange(m_nMin, m_nMax); 
		m_pWBSpinBox->setSingleStep(m_nStep);
		
		m_pWBSlider = new QSlider(Qt::Horizontal, parent);
		m_pWBSlider->setRange(m_nMin, m_nMax);
		m_pWBSlider->setSingleStep(m_nStep);

        if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_CTRL, &m_nCurrentValue) ) {
            m_pWBSlider->setValue(m_nCurrentValue);
            m_pWBSpinBox->setValue(m_nCurrentValue);
        }
		
		QObject::connect(m_pWBSlider,  SIGNAL(valueChanged(int)), m_pWBSpinBox, SLOT(setValue(int)));
        QObject::connect(m_pWBSpinBox, SIGNAL(valueChanged(int)), m_pWBSlider,  SLOT(setValue(int))); 	
        
        m_pGridLayout->addWidget(label, 26, 0, 1, 1);
        m_pGridLayout->addWidget(min, 26, 1, 1, 1);
        m_pGridLayout->addWidget(m_pWBSlider, 26, 2, 1, 1);
        m_pGridLayout->addWidget(max, 26, 3, 1, 1);
        m_pGridLayout->addWidget(m_pWBSpinBox, 26, 4, 1, 1);	
        
        connect( m_pWBSlider, SIGNAL(valueChanged(int)), this, SLOT(WB_Slider_ValueChanged(int)));
    }
	
	// WB Auto
    if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, &m_nMax, &m_nMin, &m_nStep, &m_nDefault, &m_nFlags)) {
		
		QLabel *label = new QLabel("White Balace Auto Control:");
		m_pWBAutoCheckBtn = new QCheckBox;
		//m_pWBAutoCheckBtn->setText("WB Auto Control");
		
		if( ETronDI_OK == EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, &m_nCurrentValue) ) {
			if(m_nCurrentValue == 0) m_pWBAutoCheckBtn->setChecked(false);
			else m_pWBAutoCheckBtn->setChecked(true);
		}
		
		m_pGridLayout->addWidget(label, 27, 0, 1, 1);
		m_pGridLayout->addWidget(m_pWBAutoCheckBtn, 27, 1, 1, 1);
		
		connect( m_pWBAutoCheckBtn, SIGNAL(toggled(bool)), this, SLOT(WBAuto_CheckBtn_Toggled()));
    }
    */
	
	// Processing Unit -
    
    setLayout(m_pGridLayout);   

    resize(800, 600);
    
    QScrollArea *area = new QScrollArea;
	area->setWidget(this);
    area->resize(850,600);
    area->setWindowTitle(m_pMainWindow->m_pDevInfo[m_DevSelInfo.index].strDevName);
	area->show();
}

CPropertyItems::~CPropertyItems() {
}

// EXPOSURE MODE +
void CPropertyItems::AutoMode_RadioBtn_Toggled() {
	
	if (m_pAutoModeRadioBtn->isChecked()) {
		if( ETronDI_OK != EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, 0)) {
			
			QMessageBox::critical(NULL, "Error", "Not support .", QMessageBox::Yes , QMessageBox::Yes);
			long int nValue = 0;
			EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &nValue);
			
			switch(nValue) {

				case 0:
					m_pAutoModeRadioBtn->setChecked(true);
					break;
				case 1:
					m_pManualModeRadioBtn->setChecked(true);
					break;
				case 2:
					m_pShutterModeRadioBtn->setChecked(true);
					break;
				case 3:
					m_pApertureModeRadioBtn->setChecked(true);
					break;
			}
		}
	}
}

void CPropertyItems::ManualMode_RadioBtn_Toggled() {
	
	if (m_pManualModeRadioBtn->isChecked()) {
		if( ETronDI_OK != EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, 1)) {
			
			QMessageBox::critical(NULL, "Error", "Not support .", QMessageBox::Yes , QMessageBox::Yes);
			long int nValue = 0;
			EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &nValue);
			
			switch(nValue) {

				case 0:
					m_pAutoModeRadioBtn->setChecked(true);
					break;
				case 1:
					m_pManualModeRadioBtn->setChecked(true);
					break;
				case 2:
					m_pShutterModeRadioBtn->setChecked(true);
					break;
				case 3:
					m_pApertureModeRadioBtn->setChecked(true);
					break;
			}
		}
	}
}

void CPropertyItems::ShutterMode_RadioBtn_Toggled() {
	
	if (m_pShutterModeRadioBtn->isChecked()) {
		if( ETronDI_OK != EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, 2)) {
			
			QMessageBox::critical(NULL, "Error", "Not support .", QMessageBox::Yes , QMessageBox::Yes);
			long int nValue = 0;
			EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &nValue);
			
			switch(nValue) {

				case 0:
					m_pAutoModeRadioBtn->setChecked(true);
					break;
				case 1:
					m_pManualModeRadioBtn->setChecked(true);
					break;
				case 2:
					m_pShutterModeRadioBtn->setChecked(true);
					break;
				case 3:
					m_pApertureModeRadioBtn->setChecked(true);
					break;
			}
		}
	}
}

void CPropertyItems::ApertureMode_RadioBtn_Toggled() {
	
	if (m_pApertureModeRadioBtn->isChecked()) {
		if( ETronDI_OK != EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, 3)) {
			
			QMessageBox::critical(NULL, "Error", "Not support .", QMessageBox::Yes , QMessageBox::Yes);
			long int nValue = 0;
			EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL, &nValue);
			
			switch(nValue) {

				case 0:
					m_pAutoModeRadioBtn->setChecked(true);
					break;
				case 1:
					m_pManualModeRadioBtn->setChecked(true);
					break;
				case 2:
					m_pShutterModeRadioBtn->setChecked(true);
					break;
				case 3:
					m_pApertureModeRadioBtn->setChecked(true);
					break;
			}
		}
	}
}
// EXPOSURE MODE -

// EXPOSURE PRIORITY +
void CPropertyItems::AutoExpoPriority_CheckBtn_Toggled() {
	
    if (m_pAutoExpoPriorityCheckBtn->isChecked())
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL, 1);
    else
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL, 0);
}
// EXPOSURE PRIORITY -

// EXPOSURE TIME +
void CPropertyItems::AbsExpoTime_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaExpoTime_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL, (long int)value );
}
// EXPOSURE TIME -  

// Focus +
void CPropertyItems::AbsFocus_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaFocus_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_RELATIVE_CTRL, (long int)value );
}
// Focus - 

// AUTO FOCUS +
void CPropertyItems::AutoFocus_CheckBtn_Toggled() {
	
    if (m_pAutoFocusCheckBtn->isChecked())
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_AUTO_CTRL, 1);
    else
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_FOCUS_AUTO_CTRL, 0);
} 
// AUTO FOCUS -

// IRIS +
void CPropertyItems::AbsIRIS_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaIRIS_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_IRIS_RELATIVE_CTRL, (long int)value );
}
// IRIS - 

// Room +
void CPropertyItems::AbsRoom_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaRoom_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL, (long int)value );
}
// Room - 

// Pan +
void CPropertyItems::AbsPan_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaPan_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PAN_RELATIVE_CTRL, (long int)value );
}
// Pan -

// Tilt +
void CPropertyItems::AbsTilt_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL, (long int)value );
}

void CPropertyItems::RelaTilt_Slider_ValueChanged(int value) {
	
	EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_TILT_RELATIVE_CTRL, (long int)value );
}
// Tilt -

// PRIVACY +
void CPropertyItems::Privacy_CheckBtn_Toggled() {
	
    if (m_pAutoExpoPriorityCheckBtn->isChecked())
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PRIVACY_CTRL, 1);
    else
        EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, CT_PROPERTY_ID_PRIVACY_CTRL, 0);
}
// PRIVACY -

// Backlight Compensation +
void CPropertyItems::BKLCompensation_Slider_ValueChanged(int value) {
	
	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL, (long int)value );
}
// Backlight Compensation -

// Brightness +
void CPropertyItems::Brightness_Slider_ValueChanged(int value) {
	
	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_BRIGHTNESS_CTRL, (long int)value );
}
// Brightness -

// Contrast +
void CPropertyItems::Contrast_Slider_ValueChanged(int value) {
	
	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_CONTRAST_CTRL, (long int)value );
}
// Contrast -

// Gain +
void CPropertyItems::Gain_Slider_ValueChanged(int value) {
	
	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAIN_CTRL, (long int)value );
}
// Gain -

// Power Line Freq. +
void CPropertyItems::PowerLineFreqDisable_RadioBtn_Toggled() {
	
	if (m_pPowerLineFreqDisable->isChecked()) EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, 0);
}

void CPropertyItems::PowerLineFreq50Hz_RadioBtn_Toggled() {
	
	if (m_pPowerLineFreqDisable->isChecked()) EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, 1);
}

void CPropertyItems::PowerLineFreq60Hz_RadioBtn_Toggled() {
	
	if (m_pPowerLineFreqDisable->isChecked()) EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL, 2);
}
// Power Line Freq. -

// Hue +
void CPropertyItems::Hue_Slider_ValueChanged(int value) {
	
	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_CTRL, (long int)value );
}
// Hue -

// Hue Auto +
void CPropertyItems::HueAuto_CheckBtn_Toggled() {
	
    if (m_pHueAutoCheckBtn->isChecked())
        EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_AUTO_CTRL, 1);
    else
        EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_HUE_AUTO_CTRL, 0);
}
// Hue Auto -

// Saturation +
void CPropertyItems::Saturation_Slider_ValueChanged(int value) {
	
    EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SATURATION_CTRL, (long int)value );
}
// Saturation -

// Sharpness +
void CPropertyItems::Sharpness_Slider_ValueChanged(int value) {
	
    EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_SHARPNESS_CTRL, (long int)value );
}
// Sharpness -

// Gamma +
void CPropertyItems::Gamma_Slider_ValueChanged(int value) {

	EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_GAMMA_CTRL, (long int)value );
}
// Gamma -

// WB +
void CPropertyItems::WB_Slider_ValueChanged(int value) {
	
    EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_CTRL, (long int)value );
}
// WB -

// WB FOCUS +
void CPropertyItems::WBAuto_CheckBtn_Toggled() {
	
    if (m_pWBAutoCheckBtn->isChecked())
        EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, 1);
    else
        EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL, 0);
} 
// WB FOCUS -

 
