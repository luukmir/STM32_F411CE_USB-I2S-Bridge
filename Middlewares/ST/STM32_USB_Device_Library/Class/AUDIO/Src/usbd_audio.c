/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"

extern DMA_HandleTypeDef hdma_tim2_ch1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern int32_t rxBufferGetAvailableFrames(void);
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */
#define FB_TARGET_FRAMES				512
#define FB_SAMPLE_RATE_BASE			48000U

/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq) \
  (uint8_t)(((frq * 2U * 2U) / 1000U + 4U) & 0xFFU), (uint8_t)((((frq * 2U * 2U) / 1000U) >> 8) & 0xFFU)

#ifdef USE_USBD_COMPOSITE
#define AUDIO_PACKET_SZE_WORD(frq)     (uint32_t)((((frq) * 2U * 2U)/1000U))
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */
#define AUDIO_FB_TARGET_FRAMES               ((int32_t)512)
#define AUDIO_FB_UPDATE_INTERVAL_PACKETS     ((uint16_t)2U)
#define AUDIO_FB_DEADBAND_FRAMES             ((int32_t)8)
#define AUDIO_FB_KP_NUM                       ((int32_t)3)
#define AUDIO_FB_KP_DEN                       ((int32_t)2)
#define AUDIO_FB_KI_NUM                       ((int32_t)1)
#define AUDIO_FB_KI_DEN                       ((int32_t)64)
#define AUDIO_FB_I_LIMIT_HZ                   ((int32_t)300)
#define AUDIO_FB_MAX_DEVIATION_HZ            ((int32_t)600)
#define AUDIO_FB_UNDERRUN_GUARD_FRAMES       ((int32_t)128)

typedef struct {
  uint16_t timer_diff;      // 1msあたりのカウント（47〜49）
  uint32_t raw_fb_q14;      // フィルタ後のQ14値
  int32_t  buf_error;       // バッファ残量偏差
  int32_t  trim_q14;        // 微小トリム値（-8〜+8）
  uint32_t final_fb_q14;    // 最終送信値（fb_data.rate）
  uint32_t measured_hz;     // 計算上の推定実測Hz
} AudioFeedbackDebug_t;

volatile AudioFeedbackDebug_t g_fb_debug;

typedef struct {
  int32_t target_frames;
  int32_t deadband;
  int32_t kp_num;
  int32_t kp_den;
  int32_t ki_num;
  int32_t ki_den;
  int32_t i_limit_hz;
  int32_t max_dev_hz;
  int32_t underrun_guard;
  int32_t nominal_freq;
  int32_t integral_hz;
} AudioFeedbackController_t;

static AudioFeedbackController_t g_audio_fb = {
  .target_frames   = AUDIO_FB_TARGET_FRAMES,
  .deadband        = AUDIO_FB_DEADBAND_FRAMES,
  .kp_num          = AUDIO_FB_KP_NUM,
  .kp_den          = AUDIO_FB_KP_DEN,
  .ki_num          = AUDIO_FB_KI_NUM,
  .ki_den          = AUDIO_FB_KI_DEN,
  .i_limit_hz      = AUDIO_FB_I_LIMIT_HZ,
  .max_dev_hz      = AUDIO_FB_MAX_DEVIATION_HZ,
  .underrun_guard  = AUDIO_FB_UNDERRUN_GUARD_FRAMES,
  .nominal_freq    = (int32_t)USBD_AUDIO_FREQ,
  .integral_hz     = 0,
};

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint32_t AUDIO_ComputeFeedbackFs(AudioFeedbackController_t *fb, int32_t available_frames);
static void AUDIO_UpdateFeedbackRate(USBD_AUDIO_HandleTypeDef *haudio);
static int32_t AUDIO_ApplyDeadband(int32_t frame_error, int32_t deadband);
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

/* Volume range in 1/256 dB units (UAC1 format): -80 dB .. +6 dB, 0.5 dB step. */
#define AUDIO_VOLUME_MIN_DB_256 ((int16_t)-20480)
#define AUDIO_VOLUME_MAX_DB_256 ((int16_t)1536)
#define AUDIO_VOLUME_RES_DB_256 ((int16_t)128)

/* Async feedback loop tuning for host compatibility and underrun resilience. */
#define AUDIO_FB_TARGET_FRAMES               ((int32_t)512)
#define AUDIO_FB_UPDATE_INTERVAL_PACKETS     ((uint16_t)2U)
#define AUDIO_FB_DEADBAND_FRAMES             ((int32_t)8)
#define AUDIO_FB_KP_NUM                       ((int32_t)3)
#define AUDIO_FB_KP_DEN                       ((int32_t)2)
#define AUDIO_FB_KI_NUM                       ((int32_t)1)
#define AUDIO_FB_KI_DEN                       ((int32_t)64)
#define AUDIO_FB_I_LIMIT_HZ                   ((int32_t)300)
#define AUDIO_FB_MAX_DEVIATION_HZ            ((int32_t)600)
#define AUDIO_FB_UNDERRUN_GUARD_FRAMES       ((int32_t)128)

static uint8_t audio_cur_mute = 0U;
static int16_t audio_cur_volume_db_256 = 0;

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t
    USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END = {
        /* Configuration 1 */
        0x09,                              /* bLength */
        USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
        LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), /* wTotalLength */
        HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
        0x02, /* bNumInterfaces */
        0x01, /* bConfigurationValue */
        0x00, /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
        0xC0, /* bmAttributes: Bus Powered according to user configuration */
#else
        0x80, /* bmAttributes: Bus Powered according to user configuration */
#endif                  /* USBD_SELF_POWERED */
        USBD_MAX_POWER, /* MaxPower (mA) */
        /* 09 byte*/

        /* USB Speaker Standard interface descriptor */
        AUDIO_INTERFACE_DESC_SIZE,   /* bLength */
        USB_DESC_TYPE_INTERFACE,     /* bDescriptorType */
        0x00,                        /* bInterfaceNumber */
        0x00,                        /* bAlternateSetting */
        0x00,                        /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,      /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOCONTROL, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,    /* bInterfaceProtocol */
        0x00,                        /* iInterface */
        /* 09 byte*/

        /* USB Speaker Class-specific AC Interface Descriptor */
        AUDIO_INTERFACE_DESC_SIZE,       /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_HEADER,            /* bDescriptorSubtype */
        0x00,
        /* 1.00 */ /* bcdADC */
        0x01,
        0x27, /* wTotalLength */
        0x00,
        0x01, /* bInCollection */
        0x01, /* baInterfaceNr */
        /* 09 byte*/

        /* USB Speaker Input Terminal Descriptor */
        AUDIO_INPUT_TERMINAL_DESC_SIZE,  /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_INPUT_TERMINAL,    /* bDescriptorSubtype */
        0x01,                            /* bTerminalID */
        0x01, /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
        0x01,
        0x00, /* bAssocTerminal */
        0x02, /* bNrChannels */
        0x03, /* wChannelConfig 0x0003  Stereo (L|R) */
        0x00,
        0x00, /* iChannelNames */
        0x00, /* iTerminal */
        /* 12 byte*/

        /* USB Speaker Audio Feature Unit Descriptor */
        0x09,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_FEATURE_UNIT,      /* bDescriptorSubtype */
        AUDIO_OUT_STREAMING_CTRL,        /* bUnitID */
        0x01,                            /* bSourceID */
        0x01,                            /* bControlSize */
        (AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOLUME), /* bmaControls(0) */
        0,                               /* bmaControls(1) */
        0x00,                            /* iTerminal */
        /* 09 byte */

        /* USB Speaker Output Terminal Descriptor */
        0x09,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_CONTROL_OUTPUT_TERMINAL,   /* bDescriptorSubtype */
        0x03,                            /* bTerminalID */
        0x01,                            /* wTerminalType  0x0301 */
        0x03,
        0x00, /* bAssocTerminal */
        0x02, /* bSourceID */
        0x00, /* iTerminal */
        /* 09 byte */

        /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero
           Bandwidth */
        /* Interface 1, Alternate Setting 0 */
        AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
        USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
        0x01,                          /* bInterfaceNumber */
        0x00,                          /* bAlternateSetting */
        0x00,                          /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
        0x00,                          /* iInterface */
        /* 09 byte*/

        /* USB Speaker Standard AS Interface Descriptor - Audio Streaming
           Operational */
        /* Interface 1, Alternate Setting 1 */
        AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
        USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
        0x01,                          /* bInterfaceNumber */
        0x01,                          /* bAlternateSetting */
        0x02,                          /* bNumEndpoints */
        USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
        AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
        AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
        0x00,                          /* iInterface */
        /* 09 byte*/

        /* USB Speaker Audio Streaming Interface Descriptor */
        AUDIO_STREAMING_INTERFACE_DESC_SIZE, /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
        AUDIO_STREAMING_GENERAL,             /* bDescriptorSubtype */
        0x01,                                /* bTerminalLink */
        0x01,                                /* bDelay */
        0x01, /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */
        0x00,
        /* 07 byte*/

        /* USB Speaker Audio Type III Format Interface Descriptor */
        0x0B,                            /* bLength */
        AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
        AUDIO_STREAMING_FORMAT_TYPE,     /* bDescriptorSubtype */
        AUDIO_FORMAT_TYPE_I,             /* bFormatType */
        0x02,                            /* bNrChannels */
        0x02, /* bSubFrameSize :  2 Bytes per frame (16bits) */
        16,   /* bBitResolution (16-bits per sample) */
        0x01, /* bSamFreqType only one frequency supported */
        AUDIO_SAMPLE_FREQ(
            USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
        /* 11 byte*/

        /* Endpoint 1 - Standard Descriptor */
        AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
        AUDIO_OUT_EP,                      /* bEndpointAddress 1 out endpoint */
        0x05,                              /* bmAttributes */
        AUDIO_PACKET_SZE(
            USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes
                                 (Freq(Samples)*2(Stereo)*2(HalfWord)) */
        AUDIO_FS_BINTERVAL,   /* bInterval */
        0x00,                 /* bRefresh */
        AUDIO_IN_EP,          /* bSynchAddress */
        /* 09 byte*/

        /* Endpoint - Audio Streaming Descriptor */
        AUDIO_STREAMING_ENDPOINT_DESC_SIZE, /* bLength */
        AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType */
        AUDIO_ENDPOINT_GENERAL,             /* bDescriptor */
        0x00,                               /* bmAttributes */
        0x00,                               /* bLockDelayUnits */
        0x00,                               /* wLockDelay */
        0x00,
        /* 07 byte*/

        // Endpoint 2 - Standard Descriptor - Standard AS Isochronous Synch
        // Endpoint Descriptor
        AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
        USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
        AUDIO_IN_EP,                       /* bEndpointAddress */
        0x11,                              /* bmAttributes */
        AUDIO_IN_PACKET,                   /* wMaxPacketSize in Bytes */
        0x01,                              /* bInterval 1ms */
        0x02,                              /* bRefresh 4ms = 2^2 */
        0x00,                              /* bSynchAddress */
                                           /* 09 byte*/
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t AUDIOOutEpAdd = AUDIO_OUT_EP;
static uint8_t AUDIOInEpAdd = AUDIO_IN_EP;

static inline int32_t AUDIO_ApplyDeadband(int32_t frame_error, int32_t deadband)
{
  if (frame_error > deadband) {
    return frame_error - deadband;
  }
  if (frame_error < -deadband) {
    return frame_error + deadband;
  }
  return 0;
}

static uint32_t AUDIO_ComputeFeedbackFs(AudioFeedbackController_t *fb, int32_t available_frames)
{
  int32_t fs_max = fb->nominal_freq + fb->max_dev_hz;
  int32_t fs_min = fb->nominal_freq - fb->max_dev_hz;

  if (available_frames < fb->underrun_guard) {
    fb->integral_hz = 0;
    return (uint32_t)fs_max;
  }

  const int32_t raw_error = available_frames - fb->target_frames;
  const int32_t frame_error = AUDIO_ApplyDeadband(raw_error, fb->deadband);

  const int32_t p_correction_hz = (frame_error * fb->kp_num) / fb->kp_den;
  const int32_t i_step_hz       = (frame_error * fb->ki_num) / fb->ki_den;

  int32_t i_candidate_hz = fb->integral_hz + i_step_hz;
  if (i_candidate_hz > fb->i_limit_hz) {
    i_candidate_hz = fb->i_limit_hz;
  } else if (i_candidate_hz < -fb->i_limit_hz) {
    i_candidate_hz = -fb->i_limit_hz;
  }

  int32_t correction_hz = p_correction_hz + i_candidate_hz;
  int32_t next_fs = fb->nominal_freq - correction_hz;

  if (next_fs > fs_max) {
    next_fs = fs_max;
    if (i_step_hz >= 0) {
      fb->integral_hz = i_candidate_hz;
    }
  } else if (next_fs < fs_min) {
    next_fs = fs_min;
    if (i_step_hz <= 0) {
      fb->integral_hz = i_candidate_hz;
    }
  } else {
    fb->integral_hz = i_candidate_hz;
  }

  return (uint32_t)next_fs;
}

static void AUDIO_UpdateFeedbackRate(USBD_AUDIO_HandleTypeDef *haudio)
{
  int32_t available_frames = rxBufferGetAvailableFrames();
  haudio->iso_cont.fs = AUDIO_ComputeFeedbackFs(&g_audio_fb, available_frames);
}

static uint32_t Audio_MeasureHardwareFeedbackRate(void)
{
  static uint16_t last_cnt = 0U;
  static uint32_t filtered_diff_q14 = (48U << 14); // 初期値 48.0 (Q14)
  static uint8_t init_done = 0U;

  uint16_t current_cnt = (uint16_t)(htim3.Instance->CNT);

  if (init_done == 0U) {
    last_cnt = current_cnt;
    init_done = 1U;
    return (48U << 14);
  }

  // 16bitタイマのオーバーフローを考慮した1ms差分（約48進む）
  uint16_t diff = current_cnt - last_cnt;
  last_cnt = current_cnt;

  // 異常値ガード（切断時やノイズ対策）
  if (diff < 40U || diff > 56U) {
    diff = 48U;
  }

  // IIR平滑化フィルタ: y[n] = (31 * y[n-1] + x[n]) / 32
  filtered_diff_q14 = ((filtered_diff_q14 * 31U) + ((uint32_t)diff << 14)) >> 5;

  /* --- バッファ残量による極微小トリム (音程に影響しない ±0.5 Hz 以内) --- */
  int32_t available = rxBufferGetAvailableFrames();
	int32_t buf_error = available - FB_TARGET_FRAMES; // 512からのズレ

	int32_t trim_q14 = 0;

	// 1. 目標から大きく外れている場合（起動直後や一時的な途切れ時）
	if (buf_error < -32 || buf_error > 32) {
		// 粗いP制御: 素早く512へ引き戻す (最大 ±50 Hz 程度)
		trim_q14 = -(buf_error * 16);
		if (trim_q14 > 800)  trim_q14 = 800;   // 約 +49 Hz (PCに多く送らせる)
		if (trim_q14 < -800) trim_q14 = -800;  // 約 -49 Hz (PCに少なく送らせる)
	}
	// 2. 目標近傍（512 ± 32）に収まっている定常時
	else {
		// 超微小トリム: 耳で絶対に知覚できない ±0.5 Hz 以内に固定
		trim_q14 = -(buf_error * 16) / 256;
		if (trim_q14 > 8)  trim_q14 = 8;
		if (trim_q14 < -8) trim_q14 = -8;
	}

	int32_t final_fb_q14 = (int32_t)filtered_diff_q14 + trim_q14;

  /* デバッグ用構造体への代入 */
  g_fb_debug.timer_diff   = diff;
  g_fb_debug.raw_fb_q14   = filtered_diff_q14;
  g_fb_debug.measured_hz  = (filtered_diff_q14 * 1000U) >> 14;
  g_fb_debug.buf_error    = buf_error;
  g_fb_debug.trim_q14     = trim_q14;
  g_fb_debug.final_fb_q14 = (uint32_t)final_fb_q14;

  return (uint32_t)final_fb_q14;
}
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Allocate Audio structure */
  haudio = (USBD_AUDIO_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
  }

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, AUDIOOutEpAdd, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 1U;

  /* USER CODE BEGIN */
  /* User customization: open and initialize feedback IN endpoint (async mode). */
  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 1U;

  /* Flush feedback endpoint */
  USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
  /* USER CODE END */

  haudio->alt_setting = 0U;
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->wr_ptr = 0U;
  haudio->rd_ptr = 0U;
  haudio->rd_enable = 0U;

  /* USER CODE BEGIN */
  /* User customization: initialize async feedback controller state. */
  haudio->iso_cont.fs = USBD_AUDIO_FREQ;
  haudio->iso_cont.fnsof = 0U;
  haudio->iso_cont.usbintn = 0U;
  haudio->iso_cont.tx_flag = 1U;
  haudio->iso_cont.ofs_packet = 0;

  g_audio_fb.integral_hz = 0;
  /* USER CODE END */

  /* Initialize the Audio output Hardware layer */
  if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init(USBD_AUDIO_FREQ,
                                                                      AUDIO_DEFAULT_VOLUME,
                                                                      0U) != 0U)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Prepare Out endpoint to receive 1st packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, haudio->buffer,
                               AUDIO_OUT_PACKET);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  /* USER CODE BEGIN */
  /* User customization: stop feedback transmission before endpoint close. */
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  /* USER CODE END */

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* USER CODE BEGIN */
	if (haudio != NULL) {
		haudio->iso_cont.tx_flag = 0U;
	}
	/* USER CODE END */

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, AUDIOOutEpAdd);
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 0U;
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = 0U;

  /* USER CODE BEGIN */
  /* User customization: close feedback IN endpoint. */
  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
  pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 0U;
  /* USER CODE END */

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0U);
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
          AUDIO_REQ_GetMin(pdev, req);
          break;

        case AUDIO_REQ_GET_MAX:
          AUDIO_REQ_GetMax(pdev, req);
          break;

        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetRes(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = (uint8_t *)USBD_AUDIO_GetAudioHeaderDesc(pdev->pConfDesc);
            if (pbuf != NULL)
            {
              len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
              (void)USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            /* Accept only AudioStreaming interface alt settings 0/1. */
            if (((uint8_t)(req->wIndex) == 0x01U) && ((uint8_t)(req->wValue) <= 0x01U))
            {
              haudio->alt_setting = (uint8_t)(req->wValue);
            }
            else
            {
              /* Call the error management function (command will be NAKed */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_CfgDesc);

  return USBD_AUDIO_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  /* USER CODE BEGIN */
  /* User customization: DataIn on feedback EP clears busy flag for next SOF packet. */
  USBD_AUDIO_HandleTypeDef *haudio;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL) {
    return (uint8_t)USBD_FAIL;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (epnum == (AUDIOInEpAdd & 0xFU)) {
    haudio->iso_cont.tx_flag = 0U;
  }
  /* USER CODE END */

  /* Only OUT data are processed */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      if (haudio->control.cs == AUDIO_CONTROL_MUTE)
      {
        audio_cur_mute = haudio->control.data[0];
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->MuteCtl(audio_cur_mute);
      }
      else if ((haudio->control.cs == AUDIO_CONTROL_VOLUME) && (haudio->control.len >= 2U))
      {
        int16_t vol_db_256 = (int16_t)((uint16_t)haudio->control.data[0] |
                                       ((uint16_t)haudio->control.data[1] << 8));

        if (vol_db_256 < AUDIO_VOLUME_MIN_DB_256)
        {
          vol_db_256 = AUDIO_VOLUME_MIN_DB_256;
        }
        else if (vol_db_256 > AUDIO_VOLUME_MAX_DB_256)
        {
          vol_db_256 = AUDIO_VOLUME_MAX_DB_256;
        }

        audio_cur_volume_db_256 = vol_db_256;
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->VolumeCtl(0U);
      }

      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  /* Only OUT control data are processed */
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev) {
  /* USER CODE BEGIN */
  /*
   * User customization: asynchronous feedback producer.
   * Sends 10.14 fixed-point feedback on EP IN every SOF when endpoint is idle.
   * Also aligns transmission to odd/even USB frame for robust HS scheduling.
   */
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL) {
    return (uint8_t)USBD_OK;
  }

  typedef union {
    uint32_t rate;
    uint8_t fbbuf[4];
  } R2FBB;
  R2FBB fb_data;

#ifndef USB_OTG_FS_DEVICE
#define USB_OTG_FS_DEVICE                                                      \
  ((USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE))
#endif

  if (haudio->iso_cont.tx_flag == 0U) {
    uint32_t frame_number = (USB_OTG_FS_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

    uint8_t epnum = AUDIOInEpAdd & 0x7FU;
    USB_OTG_INEndpointTypeDef *const ep =
        (USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_FS +
                                      USB_OTG_IN_ENDPOINT_BASE +
                                      (epnum * USB_OTG_EP_REG_SIZE));

    if (frame_number & 0x1U) {
      ep->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
    } else {
      ep->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
    }

    fb_data.rate = Audio_MeasureHardwareFeedbackRate();
		fb_data.fbbuf[3] = 0x00;

    if (USBD_LL_Transmit(pdev, AUDIOInEpAdd, (uint8_t *)fb_data.fbbuf, 3U) ==
        USBD_OK) {
      haudio->iso_cont.tx_flag = 1U;
    }
  }
  /* USER CODE END */

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @param  offset: audio offset
  * @retval status
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint32_t BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  haudio->offset = offset;

  if (haudio->rd_enable == 1U)
  {
    haudio->rd_ptr += (uint16_t)BufferSize;

    if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* roll back */
      haudio->rd_ptr = 0U;
    }
  }

  if (haudio->rd_ptr > haudio->wr_ptr)
  {
    if ((haudio->rd_ptr - haudio->wr_ptr) < AUDIO_OUT_PACKET)
    {
      BufferSize += 4U;
    }
    else
    {
      if ((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
      {
        BufferSize -= 4U;
      }
    }
  }
  else
  {
    if ((haudio->wr_ptr - haudio->rd_ptr) < AUDIO_OUT_PACKET)
    {
      BufferSize -= 4U;
    }
    else
    {
      if ((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
      {
        BufferSize += 4U;
      }
    }
  }

  if (haudio->offset == AUDIO_OFFSET_FULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->AudioCmd(&haudio->buffer[0],
                                                                        BufferSize, AUDIO_CMD_PLAY);
    haudio->offset = AUDIO_OFFSET_NONE;
  }
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev,
                                          uint8_t epnum) {
  /* USER CODE BEGIN */
  /* User customization: recover feedback path after IN incomplete event. */
  USBD_AUDIO_HandleTypeDef *haudio;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL) {
    return (uint8_t)USBD_FAIL;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio != NULL) {
    haudio->iso_cont.fnsof = (USB_OTG_FS_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

    if (haudio->iso_cont.tx_flag == 1U) {
      haudio->iso_cont.tx_flag = 0U;
      USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
    }
  }
  /* USER CODE END */

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev,
                                           uint8_t epnum) {
  USBD_AUDIO_HandleTypeDef *haudio;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL) {
    return (uint8_t)USBD_FAIL;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* USER CODE BEGIN */
  /* User customization: flush incomplete OUT packet before re-arming receive. */
  (void)USBD_LL_FlushEP(pdev, epnum);
  /* USER CODE END */

  /* Prepare Out endpoint to receive next audio packet */
  (void)USBD_LL_PrepareReceive(pdev, epnum, &haudio->buffer[haudio->wr_ptr],
                               AUDIO_OUT_PACKET);

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  uint16_t PacketSize;
  USBD_AUDIO_HandleTypeDef *haudio;

#ifdef USE_USBD_COMPOSITE
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL) return (uint8_t)USBD_FAIL;

  if (epnum == (AUDIOOutEpAdd & 0xFU)) {
    PacketSize = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

    /* USER CODE BEGIN */
    haudio->iso_cont.ofs_packet = (haudio->iso_cont.ofs_packet + PacketSize) % AUDIO_TOTAL_BUF_SIZE;
    /* USER CODE END */

    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])
        ->PeriodicTC(&haudio->buffer[haudio->wr_ptr], PacketSize, AUDIO_OUT_TC);

    haudio->wr_ptr += PacketSize;

    /* USER CODE BEGIN */
    haudio->iso_cont.usbintn++;

    if ((haudio->iso_cont.usbintn % AUDIO_FB_UPDATE_INTERVAL_PACKETS) == 0U) {
      AUDIO_UpdateFeedbackRate(haudio);
    }

    if ((haudio->wr_ptr + AUDIO_OUT_PACKET) > AUDIO_TOTAL_BUF_SIZE) {
      haudio->wr_ptr = 0U;
      haudio->iso_cont.usbintn = 0;
      /* USER CODE END */

      if (haudio->offset == AUDIO_OFFSET_UNKNOWN) {
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])
            ->AudioCmd(&haudio->buffer[0], AUDIO_TOTAL_BUF_SIZE / 2U, AUDIO_CMD_START);
        haudio->offset = AUDIO_OFFSET_NONE;

        /* USER CODE BEGIN */
        haudio->iso_cont.tx_flag = 0U;
        g_audio_fb.integral_hz = 0;
        /* USER CODE END */
      }
    }

    if (haudio->rd_enable == 0U) {
      if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U)) haudio->rd_enable = 1U;
    }

    (void)USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, &haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint8_t cs;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
  cs = HIBYTE(req->wValue);

  if ((HIBYTE(req->wIndex) == AUDIO_OUT_STREAMING_CTRL) && (cs == AUDIO_CONTROL_MUTE))
  {
    haudio->control.data[0] = audio_cur_mute;
  }
  else if ((HIBYTE(req->wIndex) == AUDIO_OUT_STREAMING_CTRL) && (cs == AUDIO_CONTROL_VOLUME))
  {
    haudio->control.data[0] = (uint8_t)((uint16_t)audio_cur_volume_db_256 & 0xFFU);
    haudio->control.data[1] = (uint8_t)(((uint16_t)audio_cur_volume_db_256 >> 8) & 0xFFU);
  }

  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  Handles GET_MIN Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval None
  */
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
  if ((HIBYTE(req->wIndex) == AUDIO_OUT_STREAMING_CTRL) && (HIBYTE(req->wValue) == AUDIO_CONTROL_VOLUME))
  {
    haudio->control.data[0] = (uint8_t)((uint16_t)AUDIO_VOLUME_MIN_DB_256 & 0xFFU);
    haudio->control.data[1] = (uint8_t)(((uint16_t)AUDIO_VOLUME_MIN_DB_256 >> 8) & 0xFFU);
  }

  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  Handles GET_MAX Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval None
  */
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
  if ((HIBYTE(req->wIndex) == AUDIO_OUT_STREAMING_CTRL) && (HIBYTE(req->wValue) == AUDIO_CONTROL_VOLUME))
  {
    haudio->control.data[0] = (uint8_t)((uint16_t)AUDIO_VOLUME_MAX_DB_256 & 0xFFU);
    haudio->control.data[1] = (uint8_t)(((uint16_t)AUDIO_VOLUME_MAX_DB_256 >> 8) & 0xFFU);
  }

  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  Handles GET_RES Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval None
  */
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
  if ((HIBYTE(req->wIndex) == AUDIO_OUT_STREAMING_CTRL) && (HIBYTE(req->wValue) == AUDIO_CONTROL_VOLUME))
  {
    haudio->control.data[0] = (uint8_t)((uint16_t)AUDIO_VOLUME_RES_DB_256 & 0xFFU);
    haudio->control.data[1] = (uint8_t)(((uint16_t)AUDIO_VOLUME_RES_DB_256 >> 8) & 0xFFU);
  }

  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);  /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
    haudio->control.cs = HIBYTE(req->wValue);    /* Control selector (Mute / Volume) */

    /* Prepare the reception of the buffer over EP0 */
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
  }
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_DeviceQualifierDesc);

  return USBD_AUDIO_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_AUDIO_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: Audio interface callback
  * @retval status
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

int16_t USBD_AUDIO_GetCurrentVolumeDb256(void)
{
  return audio_cur_volume_db_256;
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetEpPcktSze
  * @param  pdev: device instance (reserved for future use)
  * @param  If: Interface number (reserved for future use)
  * @param  Ep: Endpoint number (reserved for future use)
  * @retval status
  */
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep)
{
  uint32_t mps;

  UNUSED(pdev);
  UNUSED(If);
  UNUSED(Ep);

  mps = AUDIO_PACKET_SZE_WORD(USBD_AUDIO_FREQ);

  /* Return the wMaxPacketSize value in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_AUDIO_GetAudioHeaderDesc
  *         This function return the Audio descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @retval pointer to the Audio AC Header descriptor
  */
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc)
{
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  uint8_t *pAudioDesc =  NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE) &&
          (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER))
      {
        pAudioDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pAudioDesc;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */
