/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v8.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VLQ18
mcu_data: ksdk2_0
processor_version: 8.0.1
functionalGroups:
- name: RTEPIP_BasicPip
  UUID: 42ed13d3-6282-46e3-ae99-1dfb10997aed
  selectedCore: core0
- name: RTEPIP_Digital
  UUID: f3be970c-684b-4396-9c43-4a139153400a
  selectedCore: core0
- name: RTEPIP_Analog
  UUID: 4abeb533-156c-4c21-aa5e-6d334825dc52
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * RTEPIP_BasicPip functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * EDMA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'EDMA'
- type: 'edma'
- mode: 'basic'
- custom_name_enabled: 'true'
- type_id: 'edma_a23fca76a894e1bcdf9d01a687505ff9'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'DMA'
- config_sets:
  - fsl_edma:
    - common_settings:
      - enableContinuousLinkMode: 'false'
      - enableHaltOnError: 'true'
      - enableRoundRobinArbitration: 'false'
      - enableDebugMode: 'true'
    - dma_table: []
    - edma_channels: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const edma_config_t EDMA_config = {
  .enableContinuousLinkMode = false,
  .enableHaltOnError = true,
  .enableRoundRobinArbitration = false,
  .enableDebugMode = true
};

static void EDMA_init(void) {
}

/***********************************************************************************************************************
 * GPIOA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOA'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'true'
- type_id: 'gpio_be9de87e5addb6b0f416d9acbab34797'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '6'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPIOA_IRQHandler'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOA_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTA_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTA_IRQn, GPIOA_IRQ_PRIORITY);
  /* Enable interrupt PORTA_IRQn request in the NVIC */
  EnableIRQ(PORTA_IRQn);
}

/***********************************************************************************************************************
 * GPIOB initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOB'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'true'
- type_id: 'gpio_be9de87e5addb6b0f416d9acbab34797'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '6'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPIOB_IRQHandler'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOB_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTB_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTB_IRQn, GPIOB_IRQ_PRIORITY);
  /* Enable interrupt PORTB_IRQn request in the NVIC */
  EnableIRQ(PORTB_IRQn);
}

/***********************************************************************************************************************
 * GPIOC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOC'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'true'
- type_id: 'gpio_be9de87e5addb6b0f416d9acbab34797'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '6'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPIOC_IRQHandler'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOC_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTC_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTC_IRQn, GPIOC_IRQ_PRIORITY);
  /* Enable interrupt PORTC_IRQn request in the NVIC */
  EnableIRQ(PORTC_IRQn);
}

/***********************************************************************************************************************
 * GPIOD initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOD'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'true'
- type_id: 'gpio_be9de87e5addb6b0f416d9acbab34797'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '6'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPIOD_IRQHandler'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOD_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTD_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTD_IRQn, GPIOD_IRQ_PRIORITY);
  /* Enable interrupt PORTD_IRQn request in the NVIC */
  EnableIRQ(PORTD_IRQn);
}

/***********************************************************************************************************************
 * GPIOE initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOE'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'true'
- type_id: 'gpio_be9de87e5addb6b0f416d9acbab34797'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'GPIOE'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTE_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'true'
      - priority: '6'
      - enable_custom_name: 'true'
      - handler_custom_name: 'GPIOE_IRQHandler'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOE_init(void) {
  /* Make sure, the clock gate for port E is enabled (e. g. in pin_mux.c) */
  /* Interrupt vector PORTE_IRQn priority settings in the NVIC */
  NVIC_SetPriority(PORTE_IRQn, GPIOE_IRQ_PRIORITY);
  /* Enable interrupt PORTE_IRQn request in the NVIC */
  EnableIRQ(PORTE_IRQn);
}

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'true'
- type_id: 'pit_a4782ba5223c8a2527ba91aeb2bc4159'
- functional_group: 'RTEPIP_BasicPip'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'true'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
    - channels:
      - 0:
        - channel_id: ''
        - channelNumber: '0'
        - enableChain: 'false'
        - timerPeriod: '0xffffffff ticks'
        - startTimer: 'true'
        - enableInterrupt: 'false'
        - interrupt:
          - IRQn: 'PIT0_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 1:
        - channel_id: ''
        - channelNumber: '1'
        - enableChain: 'true'
        - timerMultiplier: '0xffffffff'
        - startTimer: 'true'
        - enableInterrupt: 'false'
        - interrupt:
          - IRQn: 'PIT1_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
      - 2:
        - channel_id: ''
        - channelNumber: '2'
        - enableChain: 'false'
        - timerPeriod: '1ms'
        - startTimer: 'true'
        - enableInterrupt: 'true'
        - interrupt:
          - IRQn: 'PIT2_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = true
};

static void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
  /* Set channel 0 period to N/A. */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_0, PIT_0_TICKS);
  /* Set channel 1 period to N/A. */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_1, PIT_1_TICKS);
  /* Chain the channel 1 to channel 0. */
  PIT_SetTimerChainMode(PIT_PERIPHERAL, PIT_1, true);
  /* Set channel 2 period to N/A. */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_2, PIT_2_TICKS);
  /* Enable interrupts from channel 2. */
  PIT_EnableInterrupts(PIT_PERIPHERAL, PIT_2, kPIT_TimerInterruptEnable);
  /* Enable interrupt PIT_2_IRQN request in the NVIC */
  EnableIRQ(PIT_2_IRQN);
  /* Start channel 0. */
  PIT_StartTimer(PIT_PERIPHERAL, PIT_0);
  /* Start channel 1. */
  PIT_StartTimer(PIT_PERIPHERAL, PIT_1);
  /* Start channel 2. */
  PIT_StartTimer(PIT_PERIPHERAL, PIT_2);
}

/***********************************************************************************************************************
 * RTEPIP_Digital functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * FTM0_MOTOR initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM0_MOTOR'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'FTM0'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '12000'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - chnlNumber: 'kFTM_Chnl_0'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 1:
        - chnlNumber: 'kFTM_Chnl_1'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 2:
        - chnlNumber: 'kFTM_Chnl_2'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 3:
        - chnlNumber: 'kFTM_Chnl_3'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM0_MOTOR_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_chnl_pwm_signal_param_t FTM0_MOTOR_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_0,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_1,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_2,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_3,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  }
};

static void FTM0_MOTOR_init(void) {
  FTM_Init(FTM0_MOTOR_PERIPHERAL, &FTM0_MOTOR_config);
  FTM_SetupPwm(FTM0_MOTOR_PERIPHERAL, FTM0_MOTOR_centerPwmSignalParams, sizeof(FTM0_MOTOR_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 12000U, FTM0_MOTOR_CLOCK_SOURCE);
  FTM_StartTimer(FTM0_MOTOR_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM1_ENC_L initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM1_ENC_L'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'FTM1'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM1_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '1'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadPhaseEncode'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM1_ENC_L_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t FTM1_ENC_L_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t FTM1_ENC_L_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void FTM1_ENC_L_init(void) {
  FTM_Init(FTM1_ENC_L_PERIPHERAL, &FTM1_ENC_L_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(FTM1_ENC_L_PERIPHERAL, 0,1);
  FTM_SetupQuadDecode(FTM1_ENC_L_PERIPHERAL, &FTM1_ENC_L_phaseAParams, &FTM1_ENC_L_phaseBParams, kFTM_QuadPhaseEncode);
  FTM_StartTimer(FTM1_ENC_L_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM2_ENC_R initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM2_ENC_R'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'FTM2'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM2_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '1'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadPhaseEncode'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM2_ENC_R_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t FTM2_ENC_R_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t FTM2_ENC_R_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void FTM2_ENC_R_init(void) {
  FTM_Init(FTM2_ENC_R_PERIPHERAL, &FTM2_ENC_R_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(FTM2_ENC_R_PERIPHERAL, 0,1);
  FTM_SetupQuadDecode(FTM2_ENC_R_PERIPHERAL, &FTM2_ENC_R_phaseAParams, &FTM2_ENC_R_phaseBParams, kFTM_QuadPhaseEncode);
  FTM_StartTimer(FTM2_ENC_R_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM3_SERVO initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM3_SERVO'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_04a15ae4af2b404bf2ae403c3dbe98b3'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'FTM3'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_4'
      - timerFrequency: '400'
      - bdmMode: 'kFTM_BdmMode_3'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - chnlNumber: 'kFTM_Chnl_6'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
      - 1:
        - chnlNumber: 'kFTM_Chnl_7'
        - level: 'kFTM_HighTrue'
        - dutyCyclePercent: '0'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM3_SERVO_config = {
  .prescale = kFTM_Prescale_Divide_4,
  .bdmMode = kFTM_BdmMode_3,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_chnl_pwm_signal_param_t FTM3_SERVO_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_6,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  },
  {
    .chnlNumber = kFTM_Chnl_7,
    .level = kFTM_HighTrue,
    .dutyCyclePercent = 0
  }
};

static void FTM3_SERVO_init(void) {
  FTM_Init(FTM3_SERVO_PERIPHERAL, &FTM3_SERVO_config);
  FTM_SetupPwm(FTM3_SERVO_PERIPHERAL, FTM3_SERVO_centerPwmSignalParams, sizeof(FTM3_SERVO_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 400U, FTM3_SERVO_CLOCK_SOURCE);
  FTM_StartTimer(FTM3_SERVO_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * I2C0_IMU initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C0_IMU'
- type: 'i2c'
- mode: 'I2C_Polling'
- custom_name_enabled: 'true'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '400000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t I2C0_IMU_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 400000,
  .glitchFilterWidth = 0
};

static void I2C0_IMU_init(void) {
  /* Initialization function */
  I2C_MasterInit(I2C0_IMU_PERIPHERAL, &I2C0_IMU_config, I2C0_IMU_CLK_FREQ);
}

/***********************************************************************************************************************
 * UART0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART0'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART0_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART0_init(void) {
  UART_Init(UART0_PERIPHERAL, &UART0_config, UART0_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * UART3_CAM initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART3_CAM'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'RTEPIP_Digital'
- peripheral: 'UART3'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '9600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART3_CAM_config = {
  .baudRate_Bps = 9600,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART3_CAM_init(void) {
  UART_Init(UART3_CAM_PERIPHERAL, &UART3_CAM_config, UART3_CAM_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * RTEPIP_Analog functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * ADC0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC0'
- type: 'adc16'
- mode: 'ADC'
- custom_name_enabled: 'true'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'RTEPIP_Analog'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'true'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageCount8'
    - enable_dma: 'false'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc16_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc16_config_t ADC0_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t ADC0_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t ADC0_hardwareAverageMode = kADC16_HardwareAverageCount8;

static void ADC0_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC0_PERIPHERAL, &ADC0_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(ADC0_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(ADC0_PERIPHERAL, ADC0_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC0_PERIPHERAL, ADC0_muxMode);
  /* Perform auto calibration */
  ADC16_DoAutoCalibration(ADC0_PERIPHERAL);
}

/***********************************************************************************************************************
 * ADC1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC1'
- type: 'adc16'
- mode: 'ADC'
- custom_name_enabled: 'true'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'RTEPIP_Analog'
- peripheral: 'ADC1'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'true'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageCount8'
    - enable_dma: 'false'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC1_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc16_channels_config: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc16_config_t ADC1_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t ADC1_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t ADC1_hardwareAverageMode = kADC16_HardwareAverageCount8;

static void ADC1_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC1_PERIPHERAL, &ADC1_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(ADC1_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(ADC1_PERIPHERAL, ADC1_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC1_PERIPHERAL, ADC1_muxMode);
  /* Perform auto calibration */
  ADC16_DoAutoCalibration(ADC1_PERIPHERAL);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void RTEPIP_BasicPip(void)
{
  /* Global initialization */
  DMAMUX_Init(EDMA_DMAMUX_BASEADDR);
  EDMA_Init(EDMA_DMA_BASEADDR, &EDMA_config);

  /* Initialize components */
  EDMA_init();
  GPIOA_init();
  GPIOB_init();
  GPIOC_init();
  GPIOD_init();
  GPIOE_init();
  PIT_init();
}

void RTEPIP_Digital(void)
{
  /* Initialize components */
  FTM0_MOTOR_init();
  FTM1_ENC_L_init();
  FTM2_ENC_R_init();
  FTM3_SERVO_init();
  I2C0_IMU_init();
  UART0_init();
  UART3_CAM_init();
}

void RTEPIP_Analog(void)
{
  /* Initialize components */
  ADC0_init();
  ADC1_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
}
