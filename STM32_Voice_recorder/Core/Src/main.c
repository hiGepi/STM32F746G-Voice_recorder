/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "stdio.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	F_OPEN 		= 1,
	F_CLOSE 	= 2,
	F_WRITE 	= 3,
	F_OPENDIR 	= 4,
	UNMOUNT 	= 5,
	MOUNT 		= 6,
	FORMAT		= 7,
	F_MKDIR 	= 8,
	F_CLOSEDIR  = 9,
	F_CHDIR		= 10,
	F_CHDIR_0	= 11,
}SD_Instruction;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARBG8888_BYTE_PER_PIXEL   4

/**
 * @brief  SDRAM Write read buffer start address after CAM Frame buffer
 * Assuming Camera frame buffer is of size 640x480 and format RGB565 (16 bits per pixel).
 */
#define SDRAM_WRITE_READ_ADDR        ((uint32_t)(LCD_FB_START_ADDRESS + (2*RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

#define SDRAM_WRITE_READ_ADDR_OFFSET ((uint32_t)0x0800)
#define SRAM_WRITE_READ_ADDR_OFFSET  SDRAM_WRITE_READ_ADDR_OFFSET

#define AUDIO_REC_START_ADDR         SDRAM_WRITE_READ_ADDR

#define AUDIO_BLOCK_SIZE   	((uint32_t)512)
#define AUDIO_BUFFER_IN    	AUDIO_REC_START_ADDR     /* In SDRAM */
#define AUDIO_BUFFER_OUT   	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*2)) /* In SDRAM */
#define AUDIO_BUFFER_READ  	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*4))
#define AUDIO_BUFFER_POST  	(AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE*6))

#define Audio_freq 			AUDIO_FREQUENCY_16K
#define Audio_bit_res 		DEFAULT_AUDIO_IN_BIT_RESOLUTION	//16
#define Audio_chan 			DEFAULT_AUDIO_IN_CHANNEL_NBR	//2
#define BytePerBloc			((uint16_t)Audio_bit_res*Audio_chan/8)
#define BytePerSec			((uint32_t)BytePerBloc*Audio_freq)

#define MASK_32_TO_8_0		0x000000FF
#define MASK_32_TO_8_1		0x0000FF00
#define MASK_32_TO_8_2		0x00FF0000
#define MASK_32_TO_8_3		0xFF000000

// Keyboard
#define MAJ					20
#define SPACE				'_'
#define BACKSPACE			8
#define ENTER				10

#define KEYBOARD_X 			0
#define KEYBOARD_Y			132

#define MAX_c 				30

#define GUI_width 			80
#define GUI_height			50

#define GUI_SD_x			380
#define GUI_SD_MOUNT_y		50
#define GUI_SD_UNMOUNT_y	120
#define GUI_SD_FORMAT_y		190

#define GUI_VOC_width 		40
#define GUI_VOC_height		40

#define GUI_VOC_y			220
#define GUI_VOC_PREV_x		50
#define GUI_VOC_NEXT_x		110
#define GUI_VOC_REC_x 		170
#define GUI_VOC_PLAY_x 		230

#define GUI_SAVE_x 			290
#define GUI_SAVE_width 		70

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;
SDRAM_HandleTypeDef hsdram2;

osThreadId defaultTaskHandle;
osThreadId SDHandle;
osMessageQId WakeUpHandle;
/* USER CODE BEGIN PV */

osThreadId 		inputHandle;
osMessageQId 	SD_instructionHandle;
osMessageQId 	DataHandle;
osMutexId 		mutex_LCDHandle;
osThreadId 		KBHandle;
osThreadId 		RecordHandle;

uint8_t enable = 0, fin_record = 0,etat = 0;

char 	Name[10];
char 	f_name[MAX_c+4];
char 	dir_name[16];
uint8_t keyboard[5][10] =	{
							{'1','2','3','4','5','6','7','8','9','0'},
							{'a','z','e','r','t','y','u','i','o','p'},
							{'q','s','d','f','g','h','j','k','l','m'},
							{MAJ,MAJ,'w','x','c','v','b','n',BACKSPACE,BACKSPACE},
							{0,0,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,ENTER,ENTER}
							};

char Dico[28][16] = {{"Start"},
					{"Stop"},
					{"Erreur"},
					{"Neutro"},
					{"Neutrophile"},
					{"Eosino"},
					{"Eosinophile"},
					{"Baso"},
					{"Basophile"},
					{"Lympho"},
					{"Lymphocyte"},
					{"Mono"},
					{"Monocyte"},
					{"Blaste"},
					{"Tricho"},
					{"Tricholeucocyte"},
					{"Sezary"},
					{"Autre"},
					{"Un"},
					{"Deux"},
					{"Trois"},
					{"Quatre"},
					{"Cinq"},
					{"Six"},
					{"Sept"},
					{"Huit"},
					{"Neuf"},
					{"Dix"}};

char Dico_f[28][7] = 	{{"Start"},
						{"Stop"},
						{"Erreur"},
						{"Neutro"},
						{"Neutr_"},
						{"Eosino"},
						{"Eosin_"},
						{"Baso"},
						{"Basop_"},
						{"Lympho"},
						{"Lymph_"},
						{"Mono"},
						{"Monoc_"},
						{"Blaste"},
						{"Tricho"},
						{"Trich_"},
						{"Sezary"},
						{"Autre"},
						{"Un"},
						{"Deux"},
						{"Trois"},
						{"Quatre"},
						{"Cinq"},
						{"Six"},
						{"Sept"},
						{"Huit"},
						{"Neuf"},
						{"Dix"}};

uint32_t NB_Bloc=0,Bloc_Cursor=0;
uint8_t record_OK = 0;
DIR SDDir;
Point Play[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_SAI2_Init(void);
void StartDefaultTask(void const * argument);
void StartSD(void const * argument);

/* USER CODE BEGIN PFP */
void Error_Display(int error);
void InputTask(void const * argument);
void Draw_Keyboard(uint8_t MAJ_ENABLE);
void KeyboardTask(void const * argument);
void StartRecord(void const * argument);
void SD_Init(void);
void LCD_Init(void);
void Audio_Init(void);
void write_header(uint32_t N_Bytes_Data);
void Addition(char);
void Draw_GUI(char gui, char state);
void Draw_GUI_SD(uint32_t color, char bt);
void Draw_GUI_VOC(uint32_t color, char bt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  MX_DMA2D_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  Audio_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	osMutexDef(mutex_LCD);
	mutex_LCDHandle = osMutexCreate(osMutex(mutex_LCD));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of WakeUp */
  osMessageQDef(WakeUp, 1, uint8_t);
  WakeUpHandle = osMessageCreate(osMessageQ(WakeUp), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* definition and creation of SD_instruction */
  osMessageQDef(SD_instruction, 1, SD_Instruction);
  SD_instructionHandle = osMessageCreate(osMessageQ(SD_instruction), NULL);

  /* definition and creation of Data */
  osMessageQDef(Data, 1, MAX_c);
  DataHandle = osMessageCreate(osMessageQ(Data), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SD */
  osThreadDef(SD, StartSD, osPriorityHigh, 0, 4096);
  SDHandle = osThreadCreate(osThread(SD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of Record */
  osThreadDef(Record, StartRecord, osPriorityRealtime, 0, 512);
  RecordHandle = osThreadCreate(osThread(Record), NULL);
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_SDMMC1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 255;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 255;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 132;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0xC001FE00;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 140;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_16K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.FrameInit.FrameLength = 8;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.FrameInit.FrameLength = 8;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB2.SlotInit.SlotNumber = 1;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, DCMI_PWR_EN_Pin|LED2_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_PWR_EN_Pin LED2_Pin LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin|LED2_Pin|LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D0_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
                           ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin ULPI_D2_Pin
                           ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin|ULPI_D2_Pin
                          |ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A0_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_SDA_Pin */
  GPIO_InitStruct.Pin = LCD_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(LCD_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SD_Init() {
	uint16_t rtext[_MAX_SS];/* File read buffer */

	FRESULT Res;
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler();
	} else {
		BSP_LCD_DisplayStringAt(0, 140, (uint8_t*) "SD - Mount Ok", CENTER_MODE);
		Res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
		if (Res != FR_OK) {
			BSP_LCD_DisplayStringAt(0, 150, (uint8_t*) "SD - Formatage PAS Ok",
								CENTER_MODE);
			Error_Display(Res);
		} else {
			BSP_LCD_DisplayStringAt(0, 150, (uint8_t*) "SD - Formatage Ok",
					CENTER_MODE);
//			if ((Res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0)) != FR_OK) {
//				Error_Display(Res);
//			}
		}
	}
}

void LCD_Init() {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1,
	LCD_FB_START_ADDRESS + BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetLayerVisible(0, ENABLE);
	BSP_LCD_SetLayerVisible(1, DISABLE);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);

	if (BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) != TS_OK) {
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t*) "ERROR",
				CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80,
				(uint8_t*) "Touchscreen cannot be initialized", CENTER_MODE);
	} else {
		BSP_LCD_DisplayStringAt(0, 110, (uint8_t*) "Init Ecran - OK",
				CENTER_MODE);
	}
}

void Audio_Init() {
	if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2,
	OUTPUT_DEVICE_HEADPHONE, Audio_freq,
	Audio_bit_res,
	Audio_chan) == AUDIO_OK) {
		BSP_LCD_DisplayStringAt(0, 20, (uint8_t*) "Init Audio - OK",
				CENTER_MODE);
	}

	/* Initialize SDRAM buffers */
	memset((uint16_t*) AUDIO_BUFFER_IN, 0, AUDIO_BLOCK_SIZE * 2);
	memset((uint16_t*) AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE * 2);
	memset((uint16_t*) AUDIO_BUFFER_READ, 0, AUDIO_BLOCK_SIZE * 2);

	/* Start Recording */
	BSP_AUDIO_IN_Record((uint16_t*) AUDIO_BUFFER_IN, AUDIO_BLOCK_SIZE);
	BSP_AUDIO_IN_SetVolume(190);
	BSP_AUDIO_OUT_SetVolume(70);
	/* Start Playback */
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
	if (BSP_AUDIO_OUT_Play((uint16_t*) AUDIO_BUFFER_OUT,
	AUDIO_BLOCK_SIZE * 2) == AUDIO_OK) {
		BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) "Play Audio - OK",
				CENTER_MODE);
	}

}

void write_header(uint32_t N_Bytes_Data) {
	uint32_t byteswritten;
	uint8_t entete[] = {
			0x52, 0x49, 0x46,0x46, //'RIFF' : identification du format
			(uint8_t) ((N_Bytes_Data + 36) & MASK_32_TO_8_0),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_1) >> 8),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_2) >> 16),
			(uint8_t) (((N_Bytes_Data + 36) & MASK_32_TO_8_3) >> 24), //Taille du fichier - 8 octets (en octets)

			0x57, 0x41, 0x56,0x45, //'WAVE'
			0x66, 0x6D, 0x74,0x20, //'fmt ' identifiant le format WAV
			0x10, 0x00, 0x00,0x00, //Nombre d'octets utilisés pour définir le format (16)

			0x01,0x00, //Pas de compression, format PCM entier
			Audio_chan,0x00, //Nombre de cannaux
			(uint8_t) ( Audio_freq & MASK_32_TO_8_0),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_1) >> 8),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_2) >> 16),
			(uint8_t) ((Audio_freq & MASK_32_TO_8_3) >> 24), //Fréquence d'échantillonnage

			(uint8_t) ( BytePerSec & MASK_32_TO_8_0),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_1) >> 8),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_2) >> 16),
			(uint8_t) ((BytePerSec & MASK_32_TO_8_3) >> 24), //Nombre d'octets par seconde

			(uint8_t)( BytePerBloc & MASK_32_TO_8_0),
			(uint8_t)((BytePerBloc & MASK_32_TO_8_1) >> 8), //BytePerBloc

			(Audio_bit_res),0x00, //Bits par échantillons
			0x64, 0x61, 0x74,0x61, //'DATA'
			(uint8_t) ( (N_Bytes_Data) & MASK_32_TO_8_0),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_1) >> 8),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_2) >> 16),
			(uint8_t) (((N_Bytes_Data) & MASK_32_TO_8_3) >> 24),	//DataSize
			};
	f_lseek(&SDFile, 0);
	f_write(&SDFile, entete, 44, (void*) &byteswritten);
}

void Addition(char Offset){
	if (Offset){
		for (uint16_t i = 0;i<AUDIO_BLOCK_SIZE/2;i++){
			((int16_t*)AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE/2)[i] = (((int16_t*)AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE/2)[i]+((int16_t*)AUDIO_BUFFER_READ + AUDIO_BLOCK_SIZE/2)[i])/2;
		}
	}else{
		for (uint16_t i = 0;i<AUDIO_BLOCK_SIZE/2;i++){
			((int16_t*)AUDIO_BUFFER_OUT)[i] = (((int16_t*)AUDIO_BUFFER_IN)[i]+((int16_t*)AUDIO_BUFFER_READ)[i])/2;
		}
	}
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void) {
	char a=1,b=3;
	if (enable) {
		switch(etat){
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		case 1:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE),
				(uint16_t*) (AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE),
				AUDIO_BLOCK_SIZE);
			break;
		case 2:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE),
				(uint16_t*) (AUDIO_BUFFER_IN + AUDIO_BLOCK_SIZE),
				AUDIO_BLOCK_SIZE);
			xQueueSendFromISR(WakeUpHandle,&a,0);
			break;
		case 3:
			xQueueSendFromISR(WakeUpHandle,&b,0);
			break;
		}
	}
	return;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
	char a=0,b=2;
	if (enable) {
		switch(etat){
			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		case 1:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT),
				(uint16_t*) (AUDIO_BUFFER_IN), AUDIO_BLOCK_SIZE);
			break;
		case 2:
			memcpy((uint16_t*) (AUDIO_BUFFER_OUT),
				(uint16_t*) (AUDIO_BUFFER_IN), AUDIO_BLOCK_SIZE);
			xQueueSendFromISR(WakeUpHandle,&a,0);
			break;
		case 3:
			xQueueSendFromISR(WakeUpHandle,&b,0);
			break;
		}
	}
	return;
}

void Draw_Keyboard(uint8_t MAJ_ENABLE){
	uint16_t y = 132;

	// Background
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, 132, 480, 140);

	// Keys
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	for(uint8_t i=0; i<5; i++){
		for(uint8_t j=0; j<10; j++){
			if(i<3)BSP_LCD_FillRect(j*48+2, y+i*28+2, 44, 24);
			else if(i==3){
				if(j == 0){
					if(MAJ_ENABLE)BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
					BSP_LCD_FillRect(j*48+2, y+i*28+2, 80, 24);
					if(MAJ_ENABLE)BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
				}
				if(j == 8){
					BSP_LCD_FillRect(j*48+34, y+i*28+2, 60, 24);
				}
				else if(j >= 2 && j <= 7)BSP_LCD_FillRect(j*48+2, y+i*28+2, 44, 24);
			} else {
				if(j == 8){
					BSP_LCD_FillRect(j*48+14, y+i*28+2, 80, 24);
				}
				if(j == 2)BSP_LCD_FillRect(j*48+2+30, y+i*28+2, 44+48*5-60, 24);
			}
		}
	}

	// Characters
	BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	for(uint8_t i=0; i<5; i++){
		for(uint8_t j=0; j<10; j++){
			if(keyboard[i][j] >= '0' && keyboard[i][j] <= '9')BSP_LCD_DisplayChar(j*48+2+16, y+i*28+2+4, keyboard[i][j]);
			if(keyboard[i][j] >= 'a' && keyboard[i][j] <= 'z')BSP_LCD_DisplayChar(j*48+2+16, y+i*28+2+4, keyboard[i][j]+('A'-'a')*MAJ_ENABLE);
			if(i == 3 && j == 8){
				BSP_LCD_DisplayStringAt(j*48+34+15, y+i*28+6, (uint8_t *)"DEL", LEFT_MODE);
			}
			if(i == 4 && j == 8){
				BSP_LCD_DisplayStringAt(j*48+44, y+i*28+6, (uint8_t *)"OK", LEFT_MODE);
			}
			if(i == 3 && j == 0){
				if(MAJ_ENABLE)BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
				BSP_LCD_DisplayStringAt(j*48+2+26, y+i*28+6, (uint8_t *)"MAJ", LEFT_MODE);
				if(MAJ_ENABLE)BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
			}
			if(i == 4 && j == 2){
				BSP_LCD_DisplayStringAt(230, y+i*28+2, (uint8_t *)"__", LEFT_MODE);
			}
		}
	}
}

void Draw_GUI(char gui, char state){
	SD_Instruction inst;

	switch (gui){
		case 0:
			Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 1);
			Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 2);
			Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 3);

			Play[0].X = GUI_VOC_PLAY_x+15;
			Play[0].Y = GUI_VOC_y+15;
			Play[1].X = GUI_VOC_PLAY_x+25;
			Play[1].Y = GUI_VOC_y+20;
			Play[2].X = GUI_VOC_PLAY_x+15;
			Play[2].Y = GUI_VOC_y+25;

			Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 1);
			Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 2);
			Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 3);
			Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 4);
			Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 5);
			break;

		case 1:
			switch(state){
				case 0:
					Draw_GUI_SD(LCD_COLOR_LIGHTGREEN, 1);
					inst = MOUNT;
					xQueueSend(SD_instructionHandle, &inst, 0);
					break;

				case 1:
					Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 1);
					break;

				default:
					state = 1;
					break;
			}

		case 2:
			switch(state){
				case 0:
					Draw_GUI_SD(LCD_COLOR_LIGHTGREEN, 2);
					inst = UNMOUNT;
					xQueueSend(SD_instructionHandle, &inst, 0);
					break;

				case 1:
					Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 2);
					break;

				default:
					state = 1;
					break;
			}

		case 3:
			switch(state){
				case 0:
					Draw_GUI_SD(LCD_COLOR_LIGHTGREEN, 3);
					inst = FORMAT;
					xQueueSend(SD_instructionHandle, &inst, 0);
					break;

				case 1:
					Draw_GUI_SD(LCD_COLOR_LIGHTBLUE, 3);
					break;

				default:
					state = 1;
					break;
			}

		default:
			gui = 0;
			break;
	}
}

void Draw_GUI_SD(uint32_t color, char bt){
	BSP_LCD_SetTextColor(color);

	switch(bt){
		case 1:
			BSP_LCD_FillRect(GUI_SD_x, GUI_SD_MOUNT_y, GUI_width, GUI_height);

			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_SD_x/2-6, GUI_SD_MOUNT_y+GUI_height/2-6, (uint8_t*) "MOUNT", CENTER_MODE);
			break;

		case 2:
			BSP_LCD_FillRect(GUI_SD_x, GUI_SD_UNMOUNT_y, GUI_width, GUI_height);

			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_SD_x/2-6, GUI_SD_UNMOUNT_y+GUI_height/2-6, (uint8_t*) "UNMOUNT", CENTER_MODE);
			break;

		case 3:
			BSP_LCD_FillRect(GUI_SD_x, GUI_SD_FORMAT_y, GUI_width, GUI_height);

			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_SD_x/2-6, GUI_SD_FORMAT_y+GUI_height/2-6, (uint8_t*) "FORMAT", CENTER_MODE);
			break;

		default:
			bt = 1;
			break;
	}
}

void Draw_GUI_VOC(uint32_t color, char bt){
	BSP_LCD_SetTextColor(color);

	switch(bt){
		case 1:
			BSP_LCD_FillRect(GUI_VOC_PREV_x, GUI_VOC_y, GUI_VOC_width, GUI_VOC_height);
			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_VOC_PREV_x+GUI_VOC_width/2-8, GUI_VOC_y+14, (uint8_t*) "<<", LEFT_MODE);
			break;

		case 2:
			BSP_LCD_FillRect(GUI_VOC_NEXT_x, GUI_VOC_y, GUI_VOC_width, GUI_VOC_height);
			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_VOC_NEXT_x+GUI_VOC_width/2-8, GUI_VOC_y+14, (uint8_t*) ">>", LEFT_MODE);
			break;

		case 3:
			BSP_LCD_FillRect(GUI_VOC_REC_x, GUI_VOC_y, GUI_VOC_width, GUI_VOC_height);
			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_FillRect(GUI_VOC_REC_x+15, GUI_VOC_y+15, GUI_VOC_width-30, GUI_VOC_height-30);
			break;

		case 4:
			BSP_LCD_FillRect(GUI_VOC_PLAY_x, GUI_VOC_y, GUI_VOC_width, GUI_VOC_height);
			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_FillPolygon(Play, 3);
			break;

		case 5:
			BSP_LCD_FillRect(GUI_SAVE_x, GUI_VOC_y, GUI_SAVE_width, GUI_VOC_height);
			BSP_LCD_SetBackColor(color);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_DisplayStringAt(GUI_SAVE_x+GUI_SAVE_width/2-15, GUI_VOC_y+14, (uint8_t*) "SAVE", LEFT_MODE);
			break;

		default:
			bt = 1;
			break;
	}
}
void Error_Display(int error){
	// SD error code display
	char error_code[50];
	sprintf(error_code, "SD Error : %d", error);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 70, (uint8_t *)error_code, CENTER_MODE);
	Error_Handler();
}

/* USER CODE BEGIN Header_InputTask */
/**
* @brief Function implementing the input thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputTask */
void InputTask(void const * argument)
{
  /* USER CODE BEGIN InputTask */
	uint16_t x = 20, y = 20;
	uint16_t Width = MAX_c*7;
	uint16_t Height = 12;
	uint8_t text[8] = "Name : ";
	uint16_t len = 7*strlen((char *)text);

	TS_StateTypeDef  prev_state;
	TS_StateTypeDef TS_State;

	uint8_t state = 0;
	uint8_t idx_d = 0;

	SD_Instruction inst;

	// Input area
	BSP_LCD_SetTextColor(0xFFEEEEEE);
	BSP_LCD_FillRect(x+len, y, x+Width-len, Height);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DisplayStringAt(x, y, text, LEFT_MODE);

	// GUI
	Draw_GUI(0,0);

	// Word
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(50,100, (uint8_t *) Dico[idx_d], LEFT_MODE);

	sprintf((char *)dir_name,"/data");
	inst = F_MKDIR;
	xQueueSend(SD_instructionHandle, &inst, 0);
	/* Infinite loop */
	for(;;)
	{
		// Display user name
		xSemaphoreTake(mutex_LCDHandle, portMAX_DELAY);
		BSP_LCD_SetTextColor(0xFFEEEEEE);
		BSP_LCD_FillRect(x+len, y, x+Width-len, Height);
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayStringAt(x, y, text, LEFT_MODE);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(69, 20,(uint8_t *) Name, LEFT_MODE);
		xSemaphoreGive(mutex_LCDHandle);

		// TS waiter
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected != prev_state.touchDetected && TS_State.touchDetected < 2){
			prev_state.touchDetected = TS_State.touchDetected;
			switch (state){
				// If input area touched
				case 0:
					etat = 1;
					if(TS_State.touchX[0] >= x+len && TS_State.touchX[0] <= x+Width+len &&
						TS_State.touchY[0] >= y-10 && TS_State.touchY[0] <= y+Height+10){
						state = 1;

					} else if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_MOUNT_y && TS_State.touchY[0] <= GUI_SD_MOUNT_y+GUI_height){
						state = 2;
					} else if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_UNMOUNT_y && TS_State.touchY[0] <= GUI_SD_UNMOUNT_y+GUI_height){
						state = 3;
					} else if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_FORMAT_y && TS_State.touchY[0] <= GUI_SD_FORMAT_y+GUI_height){
						state = 4;
					} else if(TS_State.touchX[0] >= GUI_VOC_PREV_x && TS_State.touchX[0] <= (GUI_VOC_PREV_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						Draw_GUI_VOC(LCD_COLOR_GRAY, 1);
						state = 5;
					} else if(TS_State.touchX[0] >= GUI_VOC_NEXT_x && TS_State.touchX[0] <= (GUI_VOC_NEXT_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						Draw_GUI_VOC(LCD_COLOR_GRAY, 2);
						state = 6;
					} else if(TS_State.touchX[0] >= GUI_VOC_REC_x && TS_State.touchX[0] <= (GUI_VOC_REC_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						state = 7;
					} else if(TS_State.touchX[0] >= GUI_VOC_PLAY_x && TS_State.touchX[0] <= (GUI_VOC_PLAY_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						state = 8;
					} else if(TS_State.touchX[0] >= GUI_SAVE_x && TS_State.touchX[0] <= (GUI_SAVE_x + GUI_SAVE_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						state = 13;
					}

					break;

				// Show keyboard
				case 1:
					if(TS_State.touchX[0] >= x+len && TS_State.touchX[0] <= x+Width+len &&
							TS_State.touchY[0] >= y-10 && TS_State.touchY[0] <= y+Height+10){
						/* definition and creation of KB */
						osThreadDef(KB, KeyboardTask, osPriorityAboveNormal, 0, 4096);
						KBHandle = osThreadCreate(osThread(KB), NULL);

						vTaskSuspend(inputHandle);
					}
					state = 0;
					break;

				// Mount
				case 2:
					if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_MOUNT_y && TS_State.touchY[0] <= GUI_SD_MOUNT_y+GUI_height){
						Draw_GUI(1,0);
						Draw_GUI(2,1);
						Draw_GUI(3,1);
					}
					state = 0;
					break;

				// Unmount
				case 3:
					if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_UNMOUNT_y && TS_State.touchY[0] <= GUI_SD_UNMOUNT_y+GUI_height){
						Draw_GUI(1,1);
						Draw_GUI(2,0);
						Draw_GUI(3,1);
					}
					state = 0;
					break;

				// Format
				case 4:
					if(TS_State.touchX[0] >= GUI_SD_x && TS_State.touchX[0] <= (GUI_SD_x + GUI_width) &&
							TS_State.touchY[0] >= GUI_SD_FORMAT_y && TS_State.touchY[0] <= GUI_SD_FORMAT_y+GUI_height){
						Draw_GUI(1,1);
						Draw_GUI(2,1);
						Draw_GUI(3,0);
					}
					state = 0;
					break;

				// Previous
				case 5:
					if(TS_State.touchX[0] >= GUI_VOC_PREV_x && TS_State.touchX[0] <= (GUI_VOC_PREV_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						if (idx_d <= 0)idx_d = 28;
						idx_d--;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
						BSP_LCD_DisplayStringAt(50,100, (uint8_t *)"                  ", LEFT_MODE);
						BSP_LCD_DisplayStringAt(50,100, (uint8_t *) Dico[idx_d], LEFT_MODE);
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 5);
					}
					Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 1);
					state = 0;
					break;

				// Next
				case 6:
					if(TS_State.touchX[0] >= GUI_VOC_NEXT_x && TS_State.touchX[0] <= (GUI_VOC_NEXT_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){
						if (idx_d >= 27)idx_d = -1;
						idx_d++;
						BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
						BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
						BSP_LCD_DisplayStringAt(50,100, (uint8_t *)"                  ", LEFT_MODE);
						BSP_LCD_DisplayStringAt(50,100, (uint8_t *) Dico[idx_d], LEFT_MODE);
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 5);
					}
					Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 2);
					state = 0;
					break;

				// First activation REC
				case 7:
					if(TS_State.touchX[0] >= GUI_VOC_REC_x && TS_State.touchX[0] <= (GUI_VOC_REC_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height){

						Draw_GUI_VOC(LCD_COLOR_RED, 3);


						sprintf((char *) f_name,"%s.wav", Dico_f[idx_d]);
						inst = F_OPEN;
						xQueueSend(SD_instructionHandle, &inst, 0);
						vTaskDelay(20);

						f_lseek(&SDFile, 44);
						NB_Bloc=0;
						etat = 2;
						state = 9;

					} else {
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 3);
						state = 0;
					}

					break;

				// First activation PLAY
				case 8:
					if(TS_State.touchX[0] >= GUI_VOC_PLAY_x && TS_State.touchX[0] <= (GUI_VOC_PLAY_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height && record_OK){
						Draw_GUI_VOC(LCD_COLOR_LIGHTGREEN, 4);
						f_lseek(&SDFile, 44);
						Bloc_Cursor=0;
						etat=3;
						state = 11;
					} else
					{
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 4);
						state = 0;
					}
					break;

				// Second touch REC
				case 9:
					if(TS_State.touchX[0] >= GUI_VOC_REC_x && TS_State.touchX[0] <= (GUI_VOC_REC_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height)
					{
						state = 10;
					} else state = 0;
					break;

				// Second activation REC
				case 10:
					if(TS_State.touchX[0] >= GUI_VOC_REC_x && TS_State.touchX[0] <= (GUI_VOC_REC_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height)
					{
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 3);
						f_lseek(&SDFile, 44);
						Bloc_Cursor=0;
						etat=1;
						record_OK=1;
					}
					state = 0;
					break;

				// Second touch PLAY
				case 11:
					if(TS_State.touchX[0] >= GUI_VOC_PLAY_x && TS_State.touchX[0] <= (GUI_VOC_PLAY_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height)
					{
						state=12;
					} else state = 0;
					break;

				// Second activation PLAY
				case 12:
					if(TS_State.touchX[0] >= GUI_VOC_PLAY_x && TS_State.touchX[0] <= (GUI_VOC_PLAY_x + GUI_VOC_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height)
					{
						Draw_GUI_VOC(LCD_COLOR_LIGHTGRAY, 4);
						etat=1;
					}
					state = 0;
					break;

				// SAVE
				case 13:
					if(TS_State.touchX[0] >= GUI_SAVE_x && TS_State.touchX[0] <= (GUI_SAVE_x + GUI_SAVE_width) &&
							TS_State.touchY[0] >= GUI_VOC_y && TS_State.touchY[0] <= GUI_VOC_y+GUI_VOC_height)
					{
						Draw_GUI_VOC(LCD_COLOR_LIGHTGREEN, 5);
						etat=0;
						if (record_OK){
							write_header(NB_Bloc*AUDIO_BLOCK_SIZE);
							f_close(&SDFile);
							record_OK = 0;
						}
						etat=1;
					}
					state = 0;
					break;

				default:
					state = 0;
					break;
			}
		}
		osDelay(20);
	}
  /* USER CODE END InputTask */
}

/* USER CODE BEGIN Header_KeyboardTask */
/**
* @brief Function implementing the KB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyboardTask */
void KeyboardTask(void const * argument)
{
	/* USER CODE BEGIN KeyboardTask */

	uint8_t state = 0, idx=strlen((char *)Name), maj_en = 0;
	uint16_t Pressed_X, Pressed_Y;
	uint16_t x = 20;
	uint16_t y = 20;
	uint16_t Width = MAX_c*7;
	uint16_t Height = 12;
	SD_Instruction instruction;
	char isDir = 0;
	TS_StateTypeDef  prev_state;
	TS_StateTypeDef TS_State;
	prev_state.touchDetected = 0;

	// Second layer activation
	BSP_LCD_SetLayerVisible(1, ENABLE);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(0);

	xSemaphoreTake(mutex_LCDHandle, portMAX_DELAY);
	Draw_Keyboard(maj_en);
	xSemaphoreGive(mutex_LCDHandle);

	/* Infinite loop */
	for(;;)
	{
		// Show typing
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(69, y,(uint8_t *) Name, LEFT_MODE);

		// TS waiter
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected != prev_state.touchDetected && TS_State.touchDetected < 2 && idx < MAX_c){
			prev_state.touchDetected = TS_State.touchDetected;

			switch (state){
				// Getting key value
				case 0:
					Pressed_X = TS_State.touchX[0]/48;
					Pressed_Y = (TS_State.touchY[0]-132)/28;
					state = 1;
					break;

				case 1:
					if(TS_State.touchX[0]/48 == Pressed_X && (TS_State.touchY[0]-132)/28 == Pressed_Y){
						state = 0;
						if(keyboard[Pressed_Y][Pressed_X] == BACKSPACE){
							if(idx>0){
								Name[idx-1] = 0;
								BSP_LCD_SetTextColor(0xFFEEEEEE);
								BSP_LCD_FillRect(69+7*(idx-1), 20, 7, 12);
								idx--;
							}

						} else if(keyboard[Pressed_Y][Pressed_X] == ENTER){
							xSemaphoreTake(mutex_LCDHandle, portMAX_DELAY);
							BSP_LCD_SetTextColor(0xFFEEEEEE);
							BSP_LCD_FillRect(x+idx*7, y, x+Width-7*7, Height);
							BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
							xSemaphoreGive(mutex_LCDHandle);


							if(strncmp((char *)Name,"circle_",7) == 0){
								BSP_LCD_DrawCircle((Name[7]-'0')*100+(Name[8]-'0')*10+(Name[9]-'0'), (Name[11]-'0')*100+(Name[12]-'0')*10+(Name[13]-'0'), (Name[15]-'0')*10+Name[16]-'0');
							} else {
								/* definition and creation of SD */
								sprintf((char *) dir_name,"/data/%s", Name);
								if(isDir == 1){
									instruction = F_CHDIR_0;
									xQueueSend(SD_instructionHandle, &instruction, 0);
								}
								instruction = F_MKDIR;
								xQueueSend(SD_instructionHandle, &instruction, 0);
								instruction = F_CHDIR;
								xQueueSend(SD_instructionHandle, &instruction, 0);
								isDir = 1;
							}
							BSP_LCD_SetLayerVisible(1, DISABLE);
							BSP_LCD_SelectLayer(0);
							vTaskResume(inputHandle);
							vTaskDelete(KBHandle);

						// Caps enable
						}else if(keyboard[Pressed_Y][Pressed_X] == MAJ){
							maj_en = !maj_en;
							Draw_Keyboard(maj_en);
						}else{
							if(keyboard[Pressed_Y][Pressed_X] >= 'A' && keyboard[Pressed_Y][Pressed_X] <= 'z')Name[idx++] = keyboard[Pressed_Y][Pressed_X]+('A'-'a')*maj_en;
							else Name[idx++] = keyboard[Pressed_Y][Pressed_X];
						}

					// If touch happened outside keyboard, deleting it
					} else if(TS_State.touchY[0] < 132){
						xSemaphoreTake(mutex_LCDHandle, portMAX_DELAY);
						BSP_LCD_SetTextColor(0xFFEEEEEE);
						BSP_LCD_FillRect(x+idx*7, y, x+Width-7*7, Height);
						BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
						xSemaphoreGive(mutex_LCDHandle);
						BSP_LCD_SetLayerVisible(1, DISABLE);
						BSP_LCD_SelectLayer(0);
						vTaskResume(inputHandle);
						vTaskDelete(KBHandle);
					}else state = 0;

					break;

				default:
					state = 0;
					break;
			}
		}
		osDelay(10);
	}
	/* USER CODE END KeyboardTask */
}
void StartRecord(void const * argument)
{
  /* USER CODE BEGIN StartRecord */
	char i;
	uint32_t byteswritten,bytesread;
  /* Infinite loop */
  for(;;)
  {
	  xQueueReceive(WakeUpHandle, &i, portMAX_DELAY);
	  if (i==0){
		  f_write(&SDFile,(uint8_t*) (AUDIO_BUFFER_OUT) , AUDIO_BLOCK_SIZE,(void*) &byteswritten);
		  NB_Bloc++;
	  }
	  if (i==1){
		  f_write(&SDFile,(uint8_t*) (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE), AUDIO_BLOCK_SIZE,(void*) &byteswritten);
		  NB_Bloc++;
	  }
	  if (i==2){
		  if (Bloc_Cursor++==NB_Bloc-1){
			  f_lseek(&SDFile, 44);
			  Bloc_Cursor=0;
		  }
		  f_read(&SDFile, ((uint8_t*)AUDIO_BUFFER_READ), AUDIO_BLOCK_SIZE,(void*) &bytesread);
		  //memcpy((uint16_t*) (AUDIO_BUFFER_OUT),(uint16_t*) (AUDIO_BUFFER_READ),AUDIO_BLOCK_SIZE);
		  Addition(0);
	  }
	  if (i==3){
		  if (Bloc_Cursor++==NB_Bloc-1){
			  f_lseek(&SDFile, 44);
			  Bloc_Cursor=0;
		  }
		  f_read(&SDFile, ((uint8_t*)AUDIO_BUFFER_READ+AUDIO_BLOCK_SIZE), AUDIO_BLOCK_SIZE,(void*) &bytesread);
		  //memcpy((uint16_t*) (AUDIO_BUFFER_OUT+AUDIO_BLOCK_SIZE),(uint16_t*) (AUDIO_BUFFER_READ+AUDIO_BLOCK_SIZE),AUDIO_BLOCK_SIZE);
		  Addition(1);
	  }
  }
  /* USER CODE END StartRecord */
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	for (;;) {
		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSD */
/**
* @brief Function implementing the SD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSD */
void StartSD(void const * argument)
{
  /* USER CODE BEGIN StartSD */
	SD_Init();
	enable = 1;

	FRESULT res; /* FatFs function common result code */
	uint16_t rtext[_MAX_SS];
	SD_Instruction instruction;

	BSP_LCD_Clear(LCD_COLOR_WHITE);

	osThreadDef(input, InputTask, osPriorityNormal, 0, 4096);
	inputHandle = osThreadCreate(osThread(input), NULL);
	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(SD_instructionHandle, &instruction, portMAX_DELAY);

		switch(instruction){
			case F_OPEN:
				if((res = f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE | FA_READ)) != FR_OK)Error_Display(res);
				break;

			case UNMOUNT:
				if ((res = f_mount(&SDFatFS, (TCHAR const*)NULL, 0)) != FR_OK)Error_Display(res);
				break;

			case MOUNT:
				if ((res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0)) != FR_OK)Error_Display(res);
				break;

			case FORMAT:
				if ((res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext))) != FR_OK)Error_Display(res);
				break;

			case F_MKDIR:
				if ((res = f_mkdir(dir_name)) != FR_OK)Error_Display(res);
				break;

			case F_OPENDIR:
				if ((res = f_opendir(&SDDir, dir_name)) != FR_OK)Error_Display(res);
				break;

			case F_CLOSEDIR:
				if ((res = f_closedir(&SDDir)) != FR_OK)Error_Display(res);
				break;

			case F_CHDIR:
				if ((res = f_chdir(dir_name)) != FR_OK)Error_Display(res);
				break;

			case F_CHDIR_0:
				if ((res = f_chdir("/data")) != FR_OK)Error_Display(res);
				break;

			default:
				break;
		}
		osDelay(10);
	}
  /* USER CODE END StartSD */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
