#include "arm_math.h"
#include "fft.h"
#include "mbed.h"

extern float32_t data_array[FFT_SIZE];   

extern float32_t fft_out[FFT_SIZE];
extern float32_t magnitude[FFT_SIZE/2];
extern float32_t SAMPLE_RATE;  // sampling rate of accelerometer

extern arm_rfft_fast_instance_f32 FFT_Instance;
extern enum symptom {NO_SYMPTOM, TREMOR, DYSKINESIA} flag;


void run_fft() {
    /* Process the data through the RFFT module */
    arm_rfft_fast_f32(&FFT_Instance, data_array, fft_out, 0);
    
    /* Process the data through the Complex Magnitude Module */
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE/2);
}

void show_results() {
    float32_t resolution = SAMPLE_RATE / FFT_SIZE;
    float32_t maxValue;
    uint32_t maxIndex;

    // find frequency with largest magnitude
    arm_max_f32(magnitude, FFT_SIZE/2, &maxValue, &maxIndex);

    printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
           maxValue, maxIndex, maxIndex * resolution);
    
    // printf("Bin\tFreq (Hz)\tMagnitude\n");
    // for (int i = 0; i < 10; i++) {  // print the first couple bins
    //     printf(" %d\t%.2f\t\t%.4f\n", i, i * resolution, magnitude[i]);
    // }
}


void detect_tremor_and_dyskinesia() {
    // circular queue to keep track of tremor/dyskinesia symptom over time
    // sampling period of 5sec results in a detection period of 5*12 = 60sec
    static int  t_arr[12] = {0}, k_arr[12] = {0};
    // avoid using heap for embedded system programming
    static CircularQueue t_queue(t_arr, 12);
    static CircularQueue k_queue(k_arr, 12);

    float32_t resolution = SAMPLE_RATE / FFT_SIZE;
    float32_t tremor_energy = 0.0f;
    float32_t dyskinesia_energy = 0.0f;
    float32_t freq;
    float32_t maxValue, maxFreq;
    uint32_t maxIndex;

    for (int i = 0; i < FFT_SIZE/2; i++) {
        freq = i * resolution;

        if (freq >= 3.0f && freq <= 5.0f) {
            tremor_energy += magnitude[i];
        }
        else if (freq > 5.0f && freq <= 7.0f) {
            dyskinesia_energy += magnitude[i];
        }
        else if (freq > 7.0f) break;
    }
    
    // find frequency with largest magnitude
    arm_max_f32(magnitude, FFT_SIZE/2, &maxValue, &maxIndex);
    maxFreq = maxIndex * resolution;

    // detected 3Hz - 5Hz as the predominant frequency, 
    // and tremor energy is above detection threshold
    if (maxFreq >= 3.0f && maxFreq <= 5.0f 
            && tremor_energy >= TREMOR_THRESHOLD) {
        t_queue.enqueue(1);
        k_queue.enqueue(0);
    }
    else if (maxFreq > 5.0f && maxFreq <= 7.0f 
            && dyskinesia_energy >= DYSKINESIA_THRESHOLD) {
        t_queue.enqueue(0);
        k_queue.enqueue(1);
    }
    else {
        t_queue.enqueue(0);
        k_queue.enqueue(0);
    }

    // update flags
    if (t_queue.get_sum() >= 9) { // 9/12 (75%) of the past 1min, tremor is detected
        flag = TREMOR; // tremor
    }
    else if (k_queue.get_sum() >= 9) {
        flag = DYSKINESIA; // dyskinesia
    }
    else {
        flag = NO_SYMPTOM; // no symptom detected
    }

    printf("t_sum: %d, k_sum: %d\n", t_queue.get_sum(), k_queue.get_sum());
}

/*
void detect_tremor_and_dyskinesia() {
    float32_t resolution = SAMPLE_RATE / FFT_SIZE;
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
*/