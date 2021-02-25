#include <M5StickC.h>
#include <math.h>
#include <driver/i2s.h>
#include "fft.h"
#include <CircularBuffer.h>

extern const unsigned char ImageData[768];

typedef struct {

    // fft step 5 と 7 (0始まり)の音の強さを保持

    int val_step5;
    int val_step7;

    // 前回サンプル値との差
    int d_step5;
    int d_step7;
} FftSample;

TFT_eSprite Disbuff = TFT_eSprite(&M5.Lcd);

void checkAXPPress()
{
	if( M5.Axp.GetBtnPress())
	{
		ESP.restart();
	}
}

#define PIN_CLK 0
#define PIN_DATA 34

bool InitI2SMicroPhone()
{
    esp_err_t err = ESP_OK;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };

    i2s_pin_config_t pin_config;
    pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = PIN_DATA;
    

    err += i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    err += i2s_set_pin(I2S_NUM_0, &pin_config);
    err += i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    //i2s_set_clk(0)

    if (err != ESP_OK)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Displaybuff()
{
    Disbuff.pushSprite(0, 0);
}

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t start_dis = NULL;
SemaphoreHandle_t start_fft = NULL;
int8_t i2s_readraw_buff[1024];
uint8_t fft_dis_buff[161][80] = {0};
uint16_t posData = 160;
CircularBuffer<FftSample, 400> fft_sample_buff;

bool CheckRing()
{
    int8_t prev_phase = 0;
    int8_t phase = 0;
    uint16_t prev_phase_index = 0;
    for (uint16_t index = 0; index < fft_sample_buff.size(); index++)
    {
        FftSample sample = fft_sample_buff[index];

        // 0: step7 立ち上がる
        // 1: 3 sample 以内に立ち下がる
        // 2: 80 sample 以内に step5 が立ち上がる
        // 3: 2 sample 以内に立ち下がる
        // 4: 120 sample 以内に立ち上がる
        // 5: 3 sample 以内に立ち下がる
        switch (phase)
        {
        case 0:
            if (sample.d_step7 > 100)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (index > 100)
            {
                phase = -1;
            }
            break;
        case 1:
            if (sample.d_step7 < -10)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (prev_phase_index + 3 < index)
            {
                phase = 0;
            }
            break;
        case 2:
            if (sample.d_step5 > 50)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (prev_phase_index + 80 < index)
            {
                phase = -1;
            }
            break;
        case 3:
            if (sample.d_step5 < -10)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (prev_phase_index + 2 < index)
            {
                phase = -1;
            }
            break;
        case 4:
            if (sample.d_step7 > 100)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (prev_phase_index + 120 < index)
            {
                phase = -1;
            }
            break;
        case 5:
            if (sample.d_step7 < -10)
            {
                phase++;
                prev_phase_index = index;
            }
            else if (prev_phase_index + 3 < index)
            {
                phase = -1;
            }
            break;
        }

        if (phase == 6)
        {
            return true;
        }

        if (phase == -1)
        {
            break;
        }
        if (phase > prev_phase)
        {
            Serial.printf("change phase: %d\n", phase);
        }

        prev_phase = phase;
    }
    return false;
}

void MicRecordfft(void *arg)
{
    int16_t *buffptr;
    size_t bytesread;
    uint16_t count_n = 0;
    float adc_data;
    double data = 0;
    uint16_t ydata;

    while (1)
    {
        xSemaphoreTake(start_fft, portMAX_DELAY);
        xSemaphoreGive(start_fft);
        fft_config_t *real_fft_plan = fft_init(512, FFT_REAL, FFT_FORWARD, NULL, NULL);
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 1024, &bytesread, (100 / portTICK_RATE_MS));
        buffptr = (int16_t *)i2s_readraw_buff;

        for (count_n = 0; count_n < real_fft_plan->size; count_n++)
        {
            adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -1000, 1000);
            real_fft_plan->input[count_n] = adc_data;
        }
        fft_execute(real_fft_plan);

        xSemaphoreTake(xSemaphore, 100 / portTICK_RATE_MS);
        FftSample sample = {
            .val_step5 = 0,
            .val_step7 = 0,
            .d_step5 = 0,
            .d_step7 = 0
        };
        for (count_n = 1; count_n < real_fft_plan->size / 4; count_n++)
        {
            data = sqrt(real_fft_plan->output[2 * count_n] * real_fft_plan->output[2 * count_n] + real_fft_plan->output[2 * count_n + 1] * real_fft_plan->output[2 * count_n + 1]);
            if ((count_n - 1) < 80)
            {
                ydata = map(data, 0, 2000, 0, 256);
                fft_dis_buff[posData][80 - count_n] = ydata;

                if (count_n == 6)
                {
                    sample.val_step5 = ydata;
                }
                else if (count_n == 8)
                {
                    sample.val_step7 = ydata;
                }
            }
        }
        if (!fft_sample_buff.isEmpty())
        {
            FftSample prev_sample = fft_sample_buff.last();
            sample.d_step5 = sample.val_step5 - prev_sample.val_step5;
            sample.d_step7 = sample.val_step7 - prev_sample.val_step7;
        }

        // full の場合は先頭を除去して末尾に追加
        if (fft_sample_buff.isFull())
        {
            fft_sample_buff.shift();
        }
        if (!fft_sample_buff.push(sample)) {
            Serial.println("Invalid push");
        }
        if (fft_sample_buff.isFull())
        {
            if (CheckRing())
            {
                fft_sample_buff.clear();
                Serial.println("CheckRing: true");
            }
        } else {
            Serial.print(".");
        }

        M5.update();
        if (M5.BtnA.wasReleased())
        {
            Serial.println("--------------------------------------");
            for (int i = 0; i < fft_sample_buff.size(); i++ ){
                FftSample s = fft_sample_buff[i];
                Serial.printf("%d,%d,%d,%d\n", s.val_step5, s.val_step7, s.d_step5, s.d_step7);
            }
        }

        posData++;
        if (posData >= 161)
        {
            posData = 0;
        }
        xSemaphoreGive(xSemaphore);
        fft_destroy(real_fft_plan);
    }
}

void Drawdisplay(void *arg)
{
    uint16_t count_x = 0, count_y = 0;
    uint16_t colorPos;
    while (1)
    {
        xSemaphoreTake(start_dis, portMAX_DELAY);
        xSemaphoreGive(start_dis);
        xSemaphoreTake(xSemaphore, 500 / portTICK_RATE_MS);
        for (count_y = 0; count_y < 80; count_y++)
        {
            for (count_x = 0; count_x < 160; count_x++)
            {
                if ((count_x + (posData % 160)) > 160)
                {
                    colorPos = fft_dis_buff[count_x + (posData % 160) - 160][count_y];
                }
                else
                {
                    colorPos = fft_dis_buff[count_x + (posData % 160)][count_y];
                }

                Disbuff.drawPixel(count_x, count_y, Disbuff.color565(ImageData[colorPos * 3 + 0], ImageData[colorPos * 3 + 1], ImageData[colorPos * 3 + 2]));
            }
        }
        xSemaphoreGive(xSemaphore);
        Disbuff.setTextColor(WHITE);
        Disbuff.setTextSize(1);
        Disbuff.fillRect(0,0,70,18,Disbuff.color565(20,20,20));
        Disbuff.drawString("MicroPhone",5,5,1);
        Disbuff.pushSprite(0, 0);
    }
}

TaskHandle_t xhandle_display = NULL;
TaskHandle_t xhandle_fft = NULL;

void setup()
{
    M5.begin();
    M5.update();

    M5.Lcd.setRotation(3);
    M5.Lcd.setSwapBytes(false);
    Disbuff.createSprite(160, 80);
    Disbuff.setSwapBytes(true);
    
    if (InitI2SMicroPhone() != true)
    {
        // ErrorMeg(0x51, "MicroPhone error",0 ,true);
    }

	int color_bk = 43;
	for (int n = 116; n < 180; n=n+8)
	{
		Disbuff.fillEllipse(80,40,n/2, n/2, Disbuff.color565(color_bk,color_bk,color_bk));
    	Displaybuff();
		delay(20);
		color_bk -= 5;
	}

	Disbuff.fillRect(0,0,160,80,BLACK);
	Displaybuff();

    xSemaphore = xSemaphoreCreateMutex();
    start_dis = xSemaphoreCreateMutex();
    start_fft = xSemaphoreCreateMutex();

    xSemaphoreTake(start_dis, portMAX_DELAY);
    xSemaphoreTake(start_fft, portMAX_DELAY);

    xTaskCreate(Drawdisplay, "Drawdisplay", 1024 * 2, (void *)0, 4, &xhandle_display);
    xTaskCreate(MicRecordfft, "MicRecordfft", 1024 * 2, (void *)0, 5, &xhandle_fft);

    xSemaphoreGive(start_dis);
    xSemaphoreGive(start_fft);
}

void loop()
{
    delay(100);

}