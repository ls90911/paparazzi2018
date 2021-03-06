#ifndef _NN_PARAMS_H
#define _NN_PARAMS_H

typedef enum {DENSE, RELU, TANH} Layer;

#define NUM_STATE_VARS 5
#define NUM_CONTROL_VARS 2
#define NUM_ALL_LAYERS 8
#define NUM_DENSE_LAYERS 4
#define MAX_LAYER_DIMS 100

extern const unsigned short LAYER_DIMS[];
extern const Layer LAYER_TYPES[];
extern const float INPUT_SCALER_PARAMS[2][NUM_STATE_VARS];
extern const float OUTPUT_SCALER_PARAMS[4][NUM_CONTROL_VARS];
extern const float BIASES[NUM_DENSE_LAYERS][MAX_LAYER_DIMS];
extern const float WEIGHTS[NUM_DENSE_LAYERS][MAX_LAYER_DIMS][MAX_LAYER_DIMS];

#endif
