/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audio_ring_buffer.hpp"

#include <array>
#include <cmath>
#include <cstdint>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
namespace AudioConfig {
constexpr uint32_t SampleRate{48000};
constexpr uint32_t Channels{2};
constexpr uint32_t FramesPerBlock{SampleRate / 1000};
constexpr uint32_t SamplesPerBlock{FramesPerBlock * Channels};
constexpr uint32_t DMABufferSize{SamplesPerBlock * 2};
constexpr uint32_t HalfDMABufferSize{DMABufferSize / 2};
constexpr uint32_t USBRingBufferSize{2048};

namespace VolumeCurve {
constexpr int16_t MinDb256{-20480};
constexpr int16_t MaxDb256{1536};
constexpr float MinDb{-80.0f};
constexpr float MaxDb{6.0f};
constexpr float LiftGamma{0.65f};

constexpr uint32_t Q15Unity{32768U};
constexpr uint32_t Q15Max{65535U};
constexpr int32_t AudioSampleMin{-32768};
constexpr int32_t AudioSampleMax{32767};
constexpr int32_t Q15RoundOffset{1 << 14};
constexpr int32_t Q15Shift{15};
} // namespace VolumeCurve

namespace LinkSync {
constexpr int16_t PatternWord{static_cast<int16_t>(0x96A5)};
constexpr uint32_t RetryChunksBeforeRestart{32};
} // namespace LinkSync
} // namespace AudioConfig

volatile uint32_t g_debug_samples_in = 0;
volatile uint32_t g_debug_samples_out = 0;
volatile uint32_t g_debug_dma_half_callbacks = 0;
volatile uint32_t g_debug_dma_full_callbacks = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
__attribute__((aligned(32))) std::array<int16_t, AudioConfig::DMABufferSize>
    txBuffer;
AudioRingBuffer<int16_t, AudioConfig::USBRingBufferSize> rxBuffer;
volatile uint32_t gStreamLockAcquired{0};
volatile uint32_t gDummyChunksWithoutLock{0};
volatile uint32_t gRequestTxRestart{0};
volatile uint32_t gH723ReadyWasAsserted{0};
volatile uint32_t gUsbGainQ15{AudioConfig::VolumeCurve::Q15Unity};
volatile uint32_t gUsbMute{0U};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void setF411Ready(bool active) {
  HAL_GPIO_WritePin(F411_READY_OUT_GPIO_Port, F411_READY_OUT_Pin,
                    active ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool restartI2STxDma() {
  auto *pTx{reinterpret_cast<uint16_t *>(txBuffer.data())};

  setF411Ready(false);
  if (HAL_I2S_DMAStop(&hi2s1) != HAL_OK) {
    return false;
  }
  if (HAL_I2S_Transmit_DMA(&hi2s1, pTx,
                           static_cast<uint16_t>(txBuffer.size())) != HAL_OK) {
    return false;
  }
  setF411Ready(true);
  return true;
}

static float clamp01(float v) {
  if (v < 0.0f) {
    return 0.0f;
  }
  if (v > 1.0f) {
    return 1.0f;
  }
  return v;
}

static uint32_t volumeDb256ToQ15(int16_t volDb256) {
  int16_t clampedDb256 = volDb256;
  if (clampedDb256 < AudioConfig::VolumeCurve::MinDb256) {
    clampedDb256 = AudioConfig::VolumeCurve::MinDb256;
  } else if (clampedDb256 > AudioConfig::VolumeCurve::MaxDb256) {
    clampedDb256 = AudioConfig::VolumeCurve::MaxDb256;
  }

  const float rawDb = static_cast<float>(clampedDb256) / 256.0f;
  const float normalized =
      (rawDb - AudioConfig::VolumeCurve::MinDb) /
      (AudioConfig::VolumeCurve::MaxDb - AudioConfig::VolumeCurve::MinDb);
  const float shaped = powf(clamp01(normalized), AudioConfig::VolumeCurve::LiftGamma);
  const float effectiveDb =
      AudioConfig::VolumeCurve::MinDb +
      (AudioConfig::VolumeCurve::MaxDb - AudioConfig::VolumeCurve::MinDb) * shaped;

  const float gain = powf(10.0f, effectiveDb / 20.0f);
  int32_t gainQ15 = static_cast<int32_t>(gain * static_cast<float>(AudioConfig::VolumeCurve::Q15Unity) + 0.5f);

  if (gainQ15 < 0) {
    gainQ15 = 0;
  } else if (gainQ15 > static_cast<int32_t>(AudioConfig::VolumeCurve::Q15Max)) {
    gainQ15 = static_cast<int32_t>(AudioConfig::VolumeCurve::Q15Max);
  }

  return static_cast<uint32_t>(gainQ15);
}

static int16_t scaleAndSaturateSampleQ15(int16_t sample, uint32_t gainQ15) {
  int32_t scaled =
      (static_cast<int32_t>(sample) * static_cast<int32_t>(gainQ15) +
       AudioConfig::VolumeCurve::Q15RoundOffset) >>
      AudioConfig::VolumeCurve::Q15Shift;

  if (scaled > AudioConfig::VolumeCurve::AudioSampleMax) {
    scaled = AudioConfig::VolumeCurve::AudioSampleMax;
  } else if (scaled < AudioConfig::VolumeCurve::AudioSampleMin) {
    scaled = AudioConfig::VolumeCurve::AudioSampleMin;
  }

  return static_cast<int16_t>(scaled);
}

static void resetLinkSyncState() {
  gStreamLockAcquired = 0U;
  gDummyChunksWithoutLock = 0U;
  gRequestTxRestart = 1U;
  rxBuffer.reset();
}

static bool updateStreamLockState(bool h723ReadyNow) {
  if (!h723ReadyNow && gH723ReadyWasAsserted != 0U) {
    resetLinkSyncState();
  }

  if (gStreamLockAcquired == 0U && h723ReadyNow) {
    gStreamLockAcquired = 1U;
    gDummyChunksWithoutLock = 0U;
  }

  gH723ReadyWasAsserted = h723ReadyNow ? 1U : 0U;
  return gStreamLockAcquired != 0U;
}

static void renderDummyAudioBlock(uint32_t start, uint32_t end) {
  ++gDummyChunksWithoutLock;
  if (gDummyChunksWithoutLock >= AudioConfig::LinkSync::RetryChunksBeforeRestart) {
    gDummyChunksWithoutLock = 0U;
    gRequestTxRestart = 1U;
  }

  for (uint32_t i = start; i < end; ++i) {
    txBuffer[i] = AudioConfig::LinkSync::PatternWord;
  }
}

static void renderUsbAudioBlock(uint32_t start, uint32_t end) {
  static std::array<int16_t, AudioConfig::HalfDMABufferSize> tempBuf;

  const uint32_t numSamples{end - start};
  rxBuffer.read(tempBuf.data(), numSamples);
  g_debug_samples_out += numSamples;

  const uint32_t gainQ15 = (gUsbMute != 0U) ? 0U : gUsbGainQ15;

  for (uint32_t i = start, j = 0; i < end; ++i, ++j) {
    txBuffer[i] = scaleAndSaturateSampleQ15(tempBuf[j], gainQ15);
  }
}

static void handleAudioBlock(uint32_t start, uint32_t end) {
  const bool h723ReadyNow = HAL_GPIO_ReadPin(H723_READY_IN_GPIO_Port,
                                             H723_READY_IN_Pin) == GPIO_PIN_SET;

  if (!updateStreamLockState(h723ReadyNow)) {
    renderDummyAudioBlock(start, end);
    return;
  }

  renderUsbAudioBlock(start, end);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2S1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  setF411Ready(false);

  auto *pTx{reinterpret_cast<uint16_t *>(txBuffer.data())};
  if (HAL_I2S_Transmit_DMA(&hi2s1, pTx,
                           static_cast<uint16_t>(txBuffer.size())) != HAL_OK) {
    Error_Handler();
  }

  setF411Ready(true);

  if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (gRequestTxRestart != 0U && gStreamLockAcquired == 0U) {
      gRequestTxRestart = 0U;
      if (!restartI2STxDma()) {
        Error_Handler();
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_SLAVE_TX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
extern "C" {
void setUsbVolumeDb256(int16_t volDb256) {
  gUsbGainQ15 = volumeDb256ToQ15(volDb256);
}

void setUsbMuteState(uint8_t mute) { gUsbMute = (mute != 0U) ? 1U : 0U; }

void rxBufferReset() {
  rxBuffer.reset();
}

int32_t rxBufferGetAvailableFrames() {
  return static_cast<int32_t>(rxBuffer.getAvailableFrames());
}
void rxBufferWrite(int16_t *data, uint32_t length) {
  if (gStreamLockAcquired == 0U) {
    return;
  }

  rxBuffer.write(data, length);
  g_debug_samples_in += length;
}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (hi2s == &hi2s1) {
    ++g_debug_dma_half_callbacks;
    handleAudioBlock(0, AudioConfig::HalfDMABufferSize);
  }
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (hi2s == &hi2s1) {
    ++g_debug_dma_full_callbacks;
    handleAudioBlock(AudioConfig::HalfDMABufferSize,
                     AudioConfig::DMABufferSize);
  }
}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
