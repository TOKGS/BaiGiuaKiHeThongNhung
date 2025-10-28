/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CODE NHOM 11 (STM32F103C8T6 + FreeRTOS + OLED SSD1306 + LM75 + MQ2)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define LM75_DEFAULT_ADDR  (0x48 << 1)
#define MQ2_ADC_CHANNEL       ADC_CHANNEL_1   
#define MQ2_LEAK_THRESHOLD    600             
#define MQ2_LPG_THRESHOLD     800             
#define MQ2_SMOKE_THRESHOLD   1200            
#define MQ2_CO_THRESHOLD      1800            
#define MQ2_MAX_VALUE         4095            
#define QUEUE_LENGTH          16

#define LED_PORT   GPIOA
#define LED_PIN    GPIO_PIN_8   

/* ===== Servo quay liên tục (FS90R/SG90 360°) ===== */
#define SERVO_MIN_US            1000   
#define SERVO_MAX_US            2000
#define SERVO_CONT_NEUTRAL_US   1500   
#define SERVO_CONT_DELTA_US      250   
#define SERVO_DRIVE_TIME_MS      450   
#define SERVO_BRAKE_MS            60   


/* Private variables */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

/* Definitions for Sensors */
osThreadId_t SensorsHandle;
const osThreadAttr_t Sensors_attributes = {
  .name = "Sensors",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue"
};
/* Definitions for oledQueue */
osMessageQueueId_t oledQueueHandle;
const osMessageQueueAttr_t oledQueue_attributes = {
  .name = "oledQueue"
};
/* Definitions for i2cMutex */
osMutexId_t i2cMutexHandle;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};
/* Definitions for adcMutex */
osMutexId_t adcMutexHandle;
const osMutexAttr_t adcMutex_attributes = {
  .name = "adcMutex"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* ==== Servo helpers (SG90 trên PA15/TIM2_CH1) ==== */
static inline void Servo_WriteUs(uint16_t us);
static inline void Servo_Enable(void);
static inline void Servo_Disable(void);
static inline void Servo_Open(void);
static inline void Servo_CloseAndDisable(void);

/* Apply helpers cho process_cmd & Sensors_Task */
static inline void ApplyFanImmediate(uint8_t on);
static inline void ApplyDoorImmediate(uint8_t open);

/* USER CODE BEGIN PV */
uint8_t lm75_addr = LM75_DEFAULT_ADDR;

/* Mirror outputs (PA6/PA7) */
volatile uint8_t fan_state  = 0;   // 1=ON, 0=OFF
volatile uint8_t door_state = 0;   // 1=OPEN, 0=CLOSED

/* Hẹn giờ (tick) cho hành vi trễ */
volatile uint32_t fan_off_at_tick   = 0;   
volatile uint32_t led_on_at_tick    = 0;   
volatile uint32_t door_close_at_tick= 0;   

/* Manual override: bất kỳ thao tác thủ công sẽ giữ trạng thái trong 60s */
volatile uint32_t manual_until_tick = 0;

static inline uint8_t ManualActive(void) {
    uint32_t now = osKernelGetTickCount();
    return (manual_until_tick != 0) && ((int32_t)(manual_until_tick - now) > 0);
}

static inline void BumpManualTimer(void) {
    uint32_t now = osKernelGetTickCount();
    if (!ManualActive()) {
        manual_until_tick = now + osKernelGetTickFreq() * 60;
    }
}
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void MX_TIM2_Init(void);

/* RTOS task functions */
void Sensors_Task(void *argument);
void UART_Task(void *argument);
void OLED_Task(void *argument);

void UART_Print(const char *msg) {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(uartMutexHandle);
}

float LM75_ReadTemp(uint8_t addr) {
    uint8_t buf[2];
    int16_t raw;

    osMutexAcquire(i2cMutexHandle, osWaitForever);
    if (HAL_I2C_Mem_Read(&hi2c1, addr, 0x00,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 1000) != HAL_OK) {
        osMutexRelease(i2cMutexHandle);
        return -1000; // lỗi I2C
    }
    osMutexRelease(i2cMutexHandle);

    raw = ((int16_t)buf[0] << 8) | buf[1];
    raw >>= 7;
    if (raw & 0x0100) raw |= 0xFE00;
    return raw * 0.5f; // LM75: 0.5°C/LSB
}

/* === UART RX (interrupt) === */
static uint8_t rx_byte;              
static char    rx_line[64];
static volatile uint8_t rx_idx = 0;

static volatile uint8_t cmd_ready = 0; 
static char    cmd_buf[64];            

static void process_cmd(char *line) {
  if (!line) return;
  while (*line == ' ' || *line == '\t') line++;
  size_t n = strlen(line);
  while (n && (line[n-1] == '\r' || line[n-1] == '\n' || line[n-1] == ' ' || line[n-1] == '\t')) line[--n] = 0;

  // Lệnh thủ công: FAN/DOOR/AUTO (mọi thao tác đều kích hoạt manual 60s)
  if (!strcasecmp(line, "FAN ON"))  { ApplyFanImmediate(1); BumpManualTimer(); return; }
  if (!strcasecmp(line, "FAN OFF")) { ApplyFanImmediate(0); BumpManualTimer(); return; }
  if (!strcasecmp(line, "FAN TOG")) { ApplyFanImmediate(!fan_state); BumpManualTimer(); return; }
  if (!strcasecmp(line, "DOOR OPEN"))  { ApplyDoorImmediate(1); BumpManualTimer(); return; }
  if (!strcasecmp(line, "DOOR CLOSE")) { ApplyDoorImmediate(0); BumpManualTimer(); return; }
  if (!strcasecmp(line, "DOOR TOG"))   { ApplyDoorImmediate(!door_state); BumpManualTimer(); return; }
  if (!strcasecmp(line, "AUTO")) { manual_until_tick = 0; return; } // về auto ngay
}

/* === MQ-2: đọc median 5 mẫu (lọc nhiễu đơn giản) === */
uint16_t MQ2_Read(void) {
    uint16_t v[5];
    for (int i=0;i<5;i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        v[i] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    for (int i=0;i<5;i++) for (int j=i+1;j<5;j++) if (v[j]<v[i]) { uint16_t t=v[i]; v[i]=v[j]; v[j]=t; }
    return v[2]; // median
}

/**
  * @brief  Application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  /* Không chạy PWM servo lúc khởi động — chỉ bật khi thật sự dùng */
  Servo_Disable();

  /* Init scheduler */
  osKernelInitialize();

  /* Create mutexes */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);
  adcMutexHandle  = osMutexNew(&adcMutex_attributes);

  /* Create queues */
  uartQueueHandle = osMessageQueueNew(QUEUE_LENGTH, sizeof(float), NULL);
  oledQueueHandle = osMessageQueueNew(QUEUE_LENGTH, sizeof(float), NULL);

  /* Create tasks */
  SensorsHandle = osThreadNew(Sensors_Task, NULL, &Sensors_attributes);
  UARTHandle    = osThreadNew(UART_Task,    NULL, &UART_attributes);
  OLEDHandle    = osThreadNew(OLED_Task,    NULL, &OLED_attributes);

  osKernelStart();
  while (1) { }
}

/* === Tasks === */
void Sensors_Task(void *argument)
{
  static uint8_t fan_auto_state = 0;   // ghi nhớ trạng thái quạt ở chế độ AUTO
  float temp;

  for(;;)
  {
    /* ---- Phím PB0/PB1 (PULLUP, nhấn = 0), cạnh xuống = nhấn ---- */
    static uint8_t pb0_prev = 1, pb1_prev = 1;
    uint8_t pb0_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    uint8_t pb1_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    if (pb0_prev == 1 && pb0_now == 0) { ApplyFanImmediate(!fan_state);  BumpManualTimer(); }
    if (pb1_prev == 1 && pb1_now == 0) { ApplyDoorImmediate(!door_state); BumpManualTimer(); }
    pb0_prev = pb0_now; pb1_prev = pb1_now;

    /* ---- Đọc cảm biến ---- */
    osMutexAcquire(adcMutexHandle, osWaitForever);
    uint16_t mq2_val = MQ2_Read();
    osMutexRelease(adcMutexHandle);

    temp = LM75_ReadTemp(lm75_addr);

    /* Gửi temp cho các queue (OLED/UART) */
    osMessageQueuePut(uartQueueHandle, &temp, 0, 10);
    osMessageQueuePut(oledQueueHandle, &temp, 0, 10);

    /* ---- Manual override: giữ nguyên outputs, bỏ hẹn giờ khi đang manual ---- */
    if (ManualActive()) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, fan_state  ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, door_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        fan_off_at_tick = 0;
        led_on_at_tick  = 0;
        door_close_at_tick = 0;
        osDelay(1000);
        continue;
    } else {
        manual_until_tick = 0; // hết manual -> trở lại auto
    }

    /* ---- Tính TSTAT theo nhiệt: 0=SAFE, 1=ABN, 2=ALERT ---- */
    int tstat_new;
    if (temp >= 32.0f)      tstat_new = 2;  // ALERT
    else if (temp > 30.0f)  tstat_new = 1;  // ABN
    else                    tstat_new = 0;  // SAFE

    static int tstat_prev = -1;
    uint32_t now = osKernelGetTickCount();
    uint32_t hz  = osKernelGetTickFreq();

    /* ---- QUẠT & LED theo chuyển trạng thái ---- */
    if (tstat_prev != -1 && tstat_new != tstat_prev) {
        if (tstat_prev == 0 && tstat_new == 1) {               
            fan_off_at_tick = 0;
            ApplyFanImmediate(1);
        }
        if (tstat_prev == 1 && tstat_new == 0) {                 
            fan_off_at_tick = now + 5 * hz;
        }
        if (tstat_prev == 1 && tstat_new == 2) {                
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
        if (tstat_prev == 2 && tstat_new == 0) {                
            led_on_at_tick  = now + 5 * hz;
            fan_off_at_tick = now + 5 * hz;
        }
        if (tstat_prev == 0 && tstat_new == 2) {                 
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        }
    }
    tstat_prev = tstat_new;

    /* ---- XỬ LÝ HẸN GIỜ (chỉ khi KHÔNG manual) ---- */
    if (fan_off_at_tick && (int32_t)(fan_off_at_tick - now) <= 0) {
        ApplyFanImmediate(0);
        fan_off_at_tick = 0;
    }
    if (led_on_at_tick && (int32_t)(led_on_at_tick - now) <= 0) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        led_on_at_tick = 0;
    }
    if (door_close_at_tick && (int32_t)(door_close_at_tick - now) <= 0) {
        ApplyDoorImmediate(0);
        door_close_at_tick = 0;
    }

    /* ---------------- CỬA AUTO (nhiệt + MQ-2, có trễ đóng 5s) ---------------*/
    uint8_t door_should_open = 0;
    if (temp >= 30.0f) {
        door_should_open = 1;
    } else {
        if (mq2_val >= MQ2_CO_THRESHOLD || mq2_val >= MQ2_SMOKE_THRESHOLD || mq2_val >= MQ2_LPG_THRESHOLD) {
            door_should_open = 1;                  
        } else if (mq2_val < MQ2_LEAK_THRESHOLD) {
            door_should_open = 0;                  
        } else {
            door_should_open = door_state;         
        }
    }

    if (door_should_open) {
        door_close_at_tick = 0;                    
        if (!door_state) ApplyDoorImmediate(1);    
    } else {
        // điều kiện đóng hợp lệ chỉ khi T<30 và MQ-2 an toàn (đã xét trên)
        if (door_state && door_close_at_tick == 0) {
            door_close_at_tick = now + 5 * hz;     
        }
    }

    /* ---------------- QUẠT AUTO (theo nhiệt / LEAK, giữ như cũ) ---------------- */
    uint8_t want_fan_on_auto = fan_auto_state;
    if (temp >= 32.0f || mq2_val >= MQ2_LEAK_THRESHOLD) want_fan_on_auto = 1;
    else if (temp < 30.0f)  want_fan_on_auto = 0;
    fan_auto_state = want_fan_on_auto;

    /* ----- Áp dụng QUẠT với trễ tắt 5s khi điều kiện an toàn đạt (temp<30 hoặc MQ2<600) ----- */
    if (want_fan_on_auto) {
        
        fan_off_at_tick = 0;
        if (!fan_state) {
            ApplyFanImmediate(1);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
    } else {
        
        if (fan_state) {
            if ( (temp < 30.0f) || (mq2_val < MQ2_LEAK_THRESHOLD) ) {
                if (fan_off_at_tick == 0) {
                    fan_off_at_tick = now + 5 * hz;
                }
            } else {
                // Nếu vẫn chưa an toàn để tắt (ví dụ vùng LEAK), huỷ hẹn
                fan_off_at_tick = 0;
            }
            // giữ trạng thái hiện tại đến khi hẹn kết thúc
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        } else {
            // quạt đã tắt, đảm bảo chân output phù hợp
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            fan_off_at_tick = 0;
        }
    }

    osDelay(200);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint8_t ch = rx_byte;

    if (ch != '\r') {
      if (ch == '\n') {
        uint32_t n = (rx_idx < sizeof(rx_line)-1) ? rx_idx : (sizeof(rx_line)-1);
        rx_line[n] = '\0';
        memcpy(cmd_buf, rx_line, n + 1);
        cmd_ready = 1;
        rx_idx = 0;
      } else {
        if (rx_idx < sizeof(rx_line)-1) rx_line[rx_idx++] = (char)ch;
      }
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

/* Khuyến nghị: nếu lỗi UART (ORE/FE...), restart RX IT */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

void UART_Task(void *argument)
{
    char msg[128];
    float temp;
    for(;;)
    {
        if (cmd_ready) { cmd_ready = 0; process_cmd(cmd_buf); }

        osMessageQueueGet(uartQueueHandle, &temp, NULL, 500);
        temp = LM75_ReadTemp(lm75_addr);
        uint16_t mq2_val = MQ2_Read();

        // TSTAT: 0=SAFE,1=ABN,2=ALERT
        int TSTAT;
        if (temp >= 32.0f) TSTAT = 2;
        else if (temp > 30.0f && temp < 32.0f) TSTAT = 1;
        else TSTAT = 0;

        // GSTAT: 4=Leak,3=CO,2=Smoke,1=LPG,0=Clean
        int GSTAT;
        if (mq2_val >= MQ2_CO_THRESHOLD)      GSTAT = 3;
        else if (mq2_val >= MQ2_SMOKE_THRESHOLD) GSTAT = 2;
        else if (mq2_val >= MQ2_LPG_THRESHOLD)   GSTAT = 1;
        else if (mq2_val >= MQ2_LEAK_THRESHOLD)  GSTAT = 4;
        else GSTAT = 0;

        int FAN  = fan_state ? 1 : 0;
        int DOOR = door_state ? 1 : 0;

        // Giây còn lại về AUTO (hiển thị cho ESP32)
        uint32_t now = osKernelGetTickCount(), hz = osKernelGetTickFreq();
        int32_t remain = (manual_until_tick && (int32_t)(manual_until_tick - now) > 0)
                         ? (int32_t)((manual_until_tick - now + hz - 1)/hz) : 0;
        if (remain > 9999) remain = 9999;

        sprintf(msg, "TEMP:%.2f,ADC:%u,TSTAT:%d,GSTAT:%d,FAN:%d,DOOR:%d,REM:%d\r\n",
                temp, mq2_val, TSTAT, GSTAT, FAN, DOOR, (int)remain);
        UART_Print(msg);

        osDelay(500);
    }
}

void OLED_Task(void *argument)
{
    float temp;

    osMutexAcquire(i2cMutexHandle, osWaitForever);
    ssd1306_Init();
    ssd1306_SetContrast(0xCF);
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    osMutexRelease(i2cMutexHandle);

    for (;;)
    {
        // Giữ khung hình mới nhất
        float dummy;
        while (osMessageQueueGet(oledQueueHandle, &dummy, NULL, 0) == osOK) { }
        osMessageQueuePut(oledQueueHandle, &temp, 0, 0);

        // Đọc trực tiếp để hiển thị
        temp = LM75_ReadTemp(lm75_addr);
        uint16_t mq2_val = MQ2_Read();

        osMutexAcquire(i2cMutexHandle, 200);
        ssd1306_Fill(Black);

        int y = 0;

        /* H1: Temperature (11x18) */
        char tbuf[20];
        if (temp > -200) snprintf(tbuf, sizeof(tbuf), "%.2f C", temp);
        else             snprintf(tbuf, sizeof(tbuf), "Temp ERR");
        ssd1306_SetCursor(0, y);
        ssd1306_WriteString(tbuf, Font_11x18, White);

        /* Nhãn nhiệt (SAFE/ABN/ALERT) */
        {
          const char *tstat = (temp >= 32.0f) ? "ALERT" : (temp > 30.0f ? "ABN" : "SAFE");
          int tw = (int)strlen(tstat)*7, boxw = tw+4; if (boxw > 40) boxw = 40;
          int x = (SSD1306_WIDTH-2)-boxw; if (x < 0) x = 0;
          int hbig = Font_11x18.height, boxh = 12, by = y + (hbig - boxh)/2;
          ssd1306_FillRectangle(x, by, x+boxw, by+boxh, White);
          ssd1306_SetCursor(x+2, by + (boxh - Font_7x10.height)/2);
          ssd1306_WriteString((char*)tstat, Font_7x10, Black);
        }

        /* Tính vị trí đều cho 3 hàng còn lại */
        y += Font_11x18.height + 2;
        int y_auto = SSD1306_HEIGHT - (Font_7x10.height + 2);
        int gap = (y_auto - y - 2*Font_7x10.height)/2; if (gap < 1) gap = 1;

        /* H2: ADC (trái) + GAS (phải) */
        {
          char abuf[24]; snprintf(abuf, sizeof(abuf), "ADC:%u", mq2_val);
          ssd1306_SetCursor(0, y);
          ssd1306_WriteString(abuf, Font_7x10, White);

          const char *gshort =
            (mq2_val >= MQ2_CO_THRESHOLD)    ? "CO"    :
            (mq2_val >= MQ2_SMOKE_THRESHOLD) ? "SMOKE" :
            (mq2_val >= MQ2_LPG_THRESHOLD)   ? "LPG"   :
            (mq2_val >= MQ2_LEAK_THRESHOLD)  ? "LEAK"  : "CLEAN";
          int gw = (int)strlen(gshort)*7, bw = gw+4; if (bw > 40) bw = 40;
          int gx = (SSD1306_WIDTH-2) - bw; if (gx < 0) gx = 0;
          ssd1306_FillRectangle(gx, y, gx+bw, y+12, White);
          ssd1306_SetCursor(gx+2, y+1);
          ssd1306_WriteString((char*)gshort, Font_7x10, Black);
        }

        /* H3: Fan / Door */
        y += Font_7x10.height + gap;
        {
          char line[28];
          snprintf(line, sizeof(line), "FN:%s  DR:%s",
                   fan_state ? "ON" : "OFF",
                   door_state ? "OPEN" : "CLOSED");
          ssd1306_SetCursor(0, y);
          ssd1306_WriteString(line, Font_7x10, White);
        }

        /* H4: Countdown / MODE AUTO */
        {
          char cdbuf[32];
          uint32_t now = osKernelGetTickCount(), hz = osKernelGetTickFreq();
          int32_t remain = (manual_until_tick && (int32_t)(manual_until_tick - now) > 0)
                           ? (int32_t)((manual_until_tick - now + hz - 1)/hz) : 0;
          if (remain > 0) { if (remain > 9999) remain = 9999;
            snprintf(cdbuf, sizeof(cdbuf), "MAN -> AUTO: %ds", (int)remain);
          } else snprintf(cdbuf, sizeof(cdbuf), "MODE: AUTO");

          ssd1306_SetCursor(0, y_auto);
          ssd1306_WriteString(cdbuf, Font_7x10, White);
        }

        ssd1306_UpdateScreen();
        osMutexRelease(i2cMutexHandle);

        osDelay(200);
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

/* GPIO init function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA6/PA7: relay fan/door (active-low), mặc định OFF/CLOSED = mức HIGH */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB0/PB1/PB2: input pull-up (phím tay) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1: PA9 (TX) AF_PP, PA10 (RX) input pull-up */
  __HAL_RCC_AFIO_CLK_ENABLE();
  GPIO_InitStruct.Pin   = GPIO_PIN_9;  // TX
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin  = GPIO_PIN_10;  // RX
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* LED nguồn (normally ON) */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}


void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;                    // neutral
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  // Remap TIM2_CH1 -> PA15, giải phóng JTAG (giữ SWD)
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_AFIO_REMAP_TIM2_PARTIAL_1();

  // PA15 AF_PP
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ==== Low-level servo driver ==== */
static volatile uint8_t servo_active = 0;

static inline void Servo_WriteUs(uint16_t us)
{
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us);
}

static inline void Servo_Enable(void)
{
  if (!servo_active) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    servo_active = 1;
  }
}

static inline void Servo_Disable(void)
{
  if (servo_active) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    servo_active = 0;
  }
}

/* ==== Servo quay liên tục: OPEN/CLOSE ==== */
static inline void Servo_Neutral(void) { Servo_WriteUs(SERVO_CONT_NEUTRAL_US); }

static inline void Servo_SpinOpen(void) {
  Servo_Enable();
#ifdef SERVO_REVERSE
  uint16_t us = SERVO_CONT_NEUTRAL_US + SERVO_CONT_DELTA_US;
#else
  uint16_t us = SERVO_CONT_NEUTRAL_US - SERVO_CONT_DELTA_US;
#endif
  Servo_WriteUs(us);
  osDelay(SERVO_DRIVE_TIME_MS);
#ifdef SERVO_REVERSE
  Servo_WriteUs(SERVO_CONT_NEUTRAL_US - 120);
#else
  Servo_WriteUs(SERVO_CONT_NEUTRAL_US + 120);
#endif
  osDelay(SERVO_BRAKE_MS);
  Servo_Neutral();
  osDelay(40);
  Servo_Disable();
}

static inline void Servo_SpinClose(void) {
  Servo_Enable();
#ifdef SERVO_REVERSE
  uint16_t us = SERVO_CONT_NEUTRAL_US - SERVO_CONT_DELTA_US;
#else
  uint16_t us = SERVO_CONT_NEUTRAL_US + SERVO_CONT_DELTA_US;
#endif
  Servo_WriteUs(us);
  osDelay(SERVO_DRIVE_TIME_MS);
#ifdef SERVO_REVERSE
  Servo_WriteUs(SERVO_CONT_NEUTRAL_US + 120);
#else
  Servo_WriteUs(SERVO_CONT_NEUTRAL_US - 120);
#endif
  osDelay(SERVO_BRAKE_MS);
  Servo_Neutral();
  osDelay(40);
  Servo_Disable();
}

/* Alias giữ tên cũ */
static inline void Servo_Open(void)            { Servo_SpinOpen();  }
static inline void Servo_CloseAndDisable(void) { Servo_SpinClose(); }

/* Apply helpers */
static inline void ApplyDoorImmediate(uint8_t open) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, open ? GPIO_PIN_RESET : GPIO_PIN_SET);
  if (open) { Servo_Open();  door_state = 1; }
  else      { Servo_CloseAndDisable(); door_state = 0; }
  osDelay(150); 
}

static inline void ApplyFanImmediate(uint8_t on) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
  fan_state = on ? 1 : 0;
  osDelay(50);
}

/* Error Handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
