#include "arm_math.h"
#include "mbed.h"

#define FFT_SIZE 1024

float32_t data_array[FFT_SIZE] = {}; // 加速度计数据

float32_t fft_out[FFT_SIZE];          // FFT输出
float32_t magnitude[FFT_SIZE / 2];    // 频率幅值，只需要前一半
float32_t SAMPLE_RATE = 100.0f;        // 采样率，比如100Hz

arm_rfft_fast_instance_f32 FFT_Instance; // FFT实例

void run_fft() {
    /* Process the data through the RFFT module */
    arm_rfft_fast_f32(&FFT_Instance, data_array, fft_out, 0);
    
    /* Process the data through the Complex Magnitude Module */
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE/2);
}

void show_results() {
    float32_t resolution = SAMPLE_RATE / FFT_SIZE; // 频率分辨率
    float32_t maxValue;
    uint32_t maxIndex;

    /* 找到最大幅值的频率 */
    arm_max_f32(magnitude, FFT_SIZE/2, &maxValue, &maxIndex);

    printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
           maxValue, maxIndex, maxIndex * resolution);
    
    printf("Bin\tFreq (Hz)\tMagnitude\n");
    for (int i = 0; i < 28; i++) { // 只打印前28个频率点
        printf(" %d\t%.2f\t\t%.4f\n", i, i * resolution, magnitude[i]);
    }
    // 你如果想专门看某个bin，比如213
    int i = 213;
    printf("Bin %d (%.2f Hz): %.3f\r\n", i, i * resolution, magnitude[i]);
}

void detect_tremor_and_dyskinesia() {
    float32_t resolution = SAMPLE_RATE / FFT_SIZE;  // 每个bin对应的频率
    float32_t tremor_energy = 0.0f;
    float32_t dyskinesia_energy = 0.0f;

    // 遍历所有bin，找出3-5Hz和5-7Hz能量
    for (int i = 0; i < FFT_SIZE/2; i++) {
        float32_t freq = i * resolution;

        if (freq >= 3.0f && freq <= 5.0f) {
            tremor_energy += magnitude[i];
        }
        else if (freq > 5.0f && freq <= 7.0f) {
            dyskinesia_energy += magnitude[i];
        }
    }

    // 设置简单的能量阈值
    const float32_t TREMOR_THRESHOLD = 1.0f;
    const float32_t DYSKINESIA_THRESHOLD = 1.0f;

    printf("\r\n--- Detection Results ---\r\n");
    printf("Tremor energy (3-5Hz): %.3f\r\n", tremor_energy);
    printf("Dyskinesia energy (5-7Hz): %.3f\r\n", dyskinesia_energy);

    if (tremor_energy > TREMOR_THRESHOLD) {
        printf("⚡ Detected: Tremor (3-5Hz)\r\n");
    } else {
        printf("No significant tremor detected.\r\n");
    }

    if (dyskinesia_energy > DYSKINESIA_THRESHOLD) {
        printf("⚡ Detected: Dyskinesia (5-7Hz)\r\n");
    } else {
        printf("No significant dyskinesia detected.\r\n");
    }
}

int main(void) {
    // 初始化FFT实例
    arm_status status = arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE);
    
    if (status != ARM_MATH_SUCCESS) {
        printf("FFT initialization failed\r\n");
        while(1);
    }
    
    run_fft();        // 执行FFT
    show_results();   // 打印分析结果
    detect_tremor_and_dyskinesia();
    
    while (true) {
        ThisThread::sleep_for(1000ms); // 循环等待
    }

    return 0;
}