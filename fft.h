#include "arm_math.h"
#include "mbed.h"

// accelerometer sampling rate is 104Hz
// data_array should contain 3 seconds of sampling data
// however, since FFT_SIZE has to be a power of 2,
// we extend the sampling time to 5 seconds,
// which takes approximately 512 samples
#define FFT_SIZE 512
#define TREMOR_THRESHOLD 30.0f
#define DYSKINESIA_THRESHOLD 30.0f

/*
    a special circular queue used for 
    tracking tremor/dyskinesia symptom over time
*/
class CircularQueue {
public:
    CircularQueue(int *c_array, float32_t *i_array, int size) : 
        c_array(c_array), i_array(i_array), size(size), 
        len(0), ind(0), sum(0), intensity(0) {}

    // enqueue a value, if queue is full then loop over
    void enqueue(int val, float32_t intn) {
        if (len == size) {
            ind = (ind + 1) % size;
            sum -= c_array[ind];
            intensity -= i_array[ind];
            sum += val;
            intensity += intn;
            c_array[ind] = val;
            i_array[ind] = intn;
        } else {
            c_array[ind] = val;
            i_array[ind++] = intn;
            sum += val;
            intensity += intn;
            ++len;
        }
    }

    int get_sum(void) { return sum; }
    
    float32_t get_intensity(void) { return intensity; }

private:
    int *c_array;           // count array
    float32_t *i_array;     // intensity array
    int size;
    int len;
    int ind;
    int sum;
    float32_t intensity;
};

void run_fft();
void show_results();
void detect_tremor_and_dyskinesia();
