





typedef struct
{
	uint16	WDV_FR_MON;		// AN00
	uint16	CUTV_P_MON;		// AN01
	uint16	WDV_RL_MON;		// AN02
	uint16	CE_MON;				// AN03
	uint16	FSP_ABS_MON;		// AN04
	uint16	IF_G1_AD_MON;		// AN05
	uint16	PDF_SIG_MON; 		// AN06
	uint16	SW4LINE_F_MON;		// AN07

	uint16	SW3LINE_E_MON;		// AN08
	uint16  VDD5_MON;			// AN09
	uint16	SW1LINE_C_MON;		// AN10
	uint16	VBATT02_MON;		// AN11
	uint16	FSP_CBS_MON;		// AN12
	uint16	PDT_5V_MON;			// AN13
	uint16	VDD3_MON;			// AN14
	uint16	V5_INT_MON;			// AN15

	uint16	SIMV_MON;			// AN16
	uint16	RELV1_MON;			// AN17
	uint16	RELV2_MON;		// AN18
	uint16	V5_EXT_MON; 		// AN19
	uint16	FLNO_MON;			// AN20
	uint16	FRNO_MON;			// AN21
	uint16	MOT_POW_MON;		// AN22
	uint16	WDV_FR_DRN_MON; 	// AN23

	uint16  U_PHASE_MON;		//AN24
	uint16	VBATT01_MON;		// AN25
	uint16	PD1_MON;			// AN26
	uint16	MOC_POW_MON;		// AN27
	uint16	SW2LINE_B_MON;		// AN28
	uint16	IF_A3_MON;			// AN29
	uint16	RELV1_DRN_MON;		// AN30
	uint16	CUTV_P_DRN_MON;		// AN31

	uint16  W_PHASE_MON; //AN32
	uint16  MPS_5V_MON;		// AN33
	uint16	SIMV_DRN_MON;		// AN34
	uint16	PDF_5V_MON;			// AN35
	uint16	RLNO_MON;			// AN36
	uint16	RRNO_MON;			// AN37
	uint16	PDT_SIG_MON;		// AN38
	uint16	V_PHASE_MON;	// AN39 xxxxxxxx

	uint16	VDD_MON;			// AN40
	uint16	PD2_MON; 			// A41
	uint16	WDV_RL_DRN_MON;		// AN42
	uint16	TEMP_MON2;			// AN43xxxxxxx
	uint16	BAT2_CURR_MON;			// AN44
	uint16	U_OUT_MON;			// AN45
	uint16	W_OUT_MON; 			// AN46
	uint16	V_OUT_MON; 			// AN47
	uint16 TEMP_MON;			// AN49
	uint16 GD_VDD5V_MON;		//AN48

} ADC_value_t;

//ADC_value_t	ADC_value;
