#pragma once

namespace polymechanon_vision {


struct DetectorSettings
{	
	bool DEBUGGING;    ///< Debugging switch. If true image windows, showing the results of scanning, will be opened.
	
	//QR parameters
	bool QR_ENABLED;    ///< Enable/Disable Qr code scanning.
	bool QR_DBG_ENABLED;    
	int QR_CANNY_PAR1;
	int QR_CANNY_PAR2;
	int QRSIDE_LENGTH;		// milimeters

	//HZL parameters
	bool HZL_ENABLED;    ///< Enable/Disable Hazardous label scanning.
	bool HZL_DBG_ENABLED;    
	int HZL_CANNY_PAR1;
	int HZL_CANNY_PAR2;
	int HZL_MATCHING_METHOD;	
	int HZL_TEMPLATE_MATCHING_METHOD;	
	bool ENABLE_COLOR_MATCHING; 
	int HZLSIDE_LENGTH;		// milimeters

	// Localizing parameters
	bool LOC_DBG_ENABLED;
	int LOCALIZING_METHOD;

	DetectorSettings(): DEBUGGING(true), 
						QR_ENABLED(true), 
						QR_DBG_ENABLED(false), 
						QR_CANNY_PAR1(100), 
						QR_CANNY_PAR2(200), 
						QRSIDE_LENGTH(182), 
						HZL_ENABLED(true), 
						HZL_DBG_ENABLED(false), 
						HZL_CANNY_PAR1(100), 
						HZL_CANNY_PAR2(200), 
						HZL_MATCHING_METHOD(0), 
						HZL_TEMPLATE_MATCHING_METHOD(4), 
						ENABLE_COLOR_MATCHING(true), 
						HZLSIDE_LENGTH(182), 
						LOC_DBG_ENABLED(false),
						LOCALIZING_METHOD(0)
						{}
};


} // "namespace polymechanon_vision"
