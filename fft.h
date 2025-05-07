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
    CircularQueue(int *array, int size) : array(array), size(size), len(0), ind(0), sum(0) {}

    // enqueue a value, if queue is full then loop over
    void enqueue(int val) {
        if (len == size) {
            ind = (ind + 1) % size;
            sum -= array[ind];
            sum += val;
            array[ind] = val;
        } else {
            array[ind++] = val;
            sum += val;
            ++len;
        }
    }

    int get_sum(void) { return sum; }

private:
    int len;
    int ind;
    int sum;
    int size;
    int *array;
};

void run_fft();
void show_results();
void detect_tremor_and_dyskinesia();
