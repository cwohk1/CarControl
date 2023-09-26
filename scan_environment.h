#ifndef SCAN_ENVIRONMENT_H
#define SCAN_ENVIRONMENT_H
#include <stdlib.h>
#include <math.h>
#define IR_VALUES
#define IR_OFFSET 7.0
#define IR_THRESHOLD 80.0
#define IR_MAX 150.0
#define FRONT_IR_MAX 150.0
#define IR_MID 90.0
#define IR_MIN 20.0
#define STOP_DISTANCE 30.0
#define NUM_SENSORS 11	// number of IR sensors

#define INITIAL_CAPACITY 12
const int make_wall_threshold = 3;
typedef struct
{
    float x, y;
} Coordinate;
typedef struct 
{
    float first;
    int second;
} Pair;

typedef struct {
    Coordinate* coordinate_data;
    Pair* pair_data;
    int data_type; // 0: coordinate, 1: pair
    size_t size;
    size_t capacity;
} Stack;

typedef struct StackStack {
    Stack* stack_data;
    size_t size;
    size_t capacity;
} StackStack;

// Initialize a stack with a given initial capacity
Stack* createStack(size_t initialCapacity, int data_type) {
    Stack* stack = (Stack*)malloc(sizeof(Stack));
    stack->data_type = data_type;
    if(data_type == 0)
        stack->coordinate_data = (Coordinate*)malloc(initialCapacity * sizeof(Coordinate));
    else if(data_type == 1)
        stack->pair_data = (Pair*)malloc(initialCapacity * sizeof(Pair));
    else
        stack->pair_data = (Pair*)malloc(initialCapacity * sizeof(Pair));
    stack->size = 0;
    stack->capacity = initialCapacity;
    return stack;
}

StackStack* createStackStack(size_t initialCapacity) {
    StackStack* stackstack = (StackStack*)malloc(sizeof(StackStack));
    stackstack->stack_data = (Stack*)malloc(initialCapacity * sizeof(Stack));
    stackstack->size = 0;
    stackstack->capacity = initialCapacity;
    return stackstack;
}

// Push an element onto the stack
void stackpush(Stack* stack, void* value) {
    if (stack->size == stack->capacity) {
        // Double the capacity if the stack is full
        stack->capacity *= 2;
        if(stack->data_type == 0)
            stack->coordinate_data = (Coordinate*)realloc(stack->coordinate_data, stack->capacity * sizeof(Coordinate));
        else if(stack->data_type == 1)
            stack->pair_data = (Pair*)realloc(stack->pair_data, stack->capacity * sizeof(Pair));
        else
            stack->pair_data = (Pair*)realloc(stack->pair_data, stack->capacity * sizeof(Pair));
    }
    if (stack->data_type == 0)
        stack->coordinate_data[stack->size++] = *(Coordinate*)value;
    else if (stack->data_type == 1)
        stack->pair_data[stack->size++] = *(Pair*)value;
    else
        stack->pair_data[stack->size++] = *(Pair*)value;
}

void stackstackpush(StackStack* stack, Stack* value) {
    if (stack->size == stack->capacity) {
        // Double the capacity if the stack is full
        stack->capacity *= 2;
        stack->stack_data = (Stack*)realloc(stack->stack_data, stack->capacity * sizeof(Stack));
    }
    stack->stack_data[stack->size++] = *value;
}

// Access an element at a specific index
void stackget(Stack* stack, size_t index, void* output) {
    if(stack->data_type == 0)
        *(Coordinate*)output = stack->coordinate_data[index];
    else if(stack->data_type == 1)
        *(Pair*)output = stack->pair_data[index];
    else
        *(Pair*)output = stack->pair_data[index];
    return;
}

void stackstackget(StackStack* stack, size_t index, Stack* output) {
    *output = stack->stack_data[index];
    return;
}
void stackpop(Stack* stack, void* output) {
    if(stack->data_type == 0)
        *(Coordinate*)output = stack->coordinate_data[--stack->size];
    else if(stack->data_type == 1)
        *(Pair*)output = stack->pair_data[--stack->size];
    else
        *(Pair*)output = stack->pair_data[--stack->size];
    return;
}

// Clear all elements from the stack
void stackclear(Stack* stack) {
    stack->size = 0;
}
size_t stacksize(Stack* stack) {
    return stack->size;
}

size_t stackstacksize(StackStack* stack) {
    return stack->size;
}

// Free the memory used by the stack
void destroyStack(Stack* stack) {
    if(stack->data_type == 0)
        free(stack->coordinate_data);
    else if(stack->data_type == 1)
        free(stack->pair_data);
    else
        free(stack->pair_data);
    free(stack);
}

void stackstackclear(StackStack* stack) {
    for(int i; i < stack->size; ++i) {
        destroyStack(&(stack->stack_data[i]));
    }
    stack->size = 0;
}

void stackstackpop(StackStack* stack, Stack* output) {
    *output = stack->stack_data[--stack->size];
    destroyStack(&(stack->stack_data[stack->size]));
    return;
}

void destroyStackStack(StackStack* stack) {
    for(int i; i < stack->size; ++i) {
        destroyStack(&(stack->stack_data[i]));
    }
    free(stack->stack_data);
    free(stack);
}
float getDistance(Coordinate P, Coordinate Ps, Coordinate Pe){
    float dist;
    if(Ps.x == Pe.x && Ps.y == Pe.y)
        dist = sqrt(pow(P.x - Ps.x, 2) + pow(P.y - Ps.y, 2));
    else{
        float parallelogram_area = fabs((Pe.x - Ps.x) * (P.y - Ps.y) - (P.x - Ps.x) * (Pe.y - Ps.y));
        float base = sqrt(pow(Ps.x - Pe.x, 2) + pow(Ps.y - Pe.y, 2));
        dist = parallelogram_area / base;
    }
    return dist;
}
Pair GetMostDistant(Stack* coords){
    float dmax = 0.0;
    int index = -1;
    for(int i=1; i < stacksize(coords)-1; i++) {
        float d = getDistance(coords->coordinate_data[i], coords->coordinate_data[0], coords->coordinate_data[stacksize(coords)-1]);
        if(d > dmax) {
            dmax = d;
            index = i;
        }
    }
    Pair result;
    result.first = dmax;
    result.second = index;
    return result;
}
StackStack* runSplitAndMerge(Stack* coords) {
    StackStack* result = createStackStack(INITIAL_CAPACITY), *result_left, *result_right;
    Pair most_distant = GetMostDistant(coords);
    float d = most_distant.first;
    int idx = most_distant.second;
    if(d > make_wall_threshold) {
        Stack* points_left = createStack(idx+1, 0);
        Stack* points_right = createStack(stacksize(coords)-idx+1, 0);
        for (int i = 0; i <= idx; ++i)
            stackpush(points_left, &(coords->coordinate_data[i]));
        for (int i = idx + 1; i < stacksize(coords); ++i)
            stackpush(points_right, &(coords->coordinate_data[i]));
        result_left = runSplitAndMerge(points_left);
        result_right = runSplitAndMerge(points_right);
        for (int i = 0; i < stackstacksize(result_left); ++i)
            stackstackpush(result, &(result_left->stack_data[i]));
        for (int i = 0; i < stackstacksize(result_right); ++i)
            stackstackpush(result, &(result_right->stack_data[i]));
        destroyStack(points_left);
        destroyStack(points_right);
    }
    else {
        stackstackpush(result, coords);
    }
    destroyStackStack(result_left);
    destroyStackStack(result_right);
    return result;
}

StackStack* grabData(float* ir_in){
    Stack* group =  createStack(INITIAL_CAPACITY, 0);
    StackStack* groups = createStackStack(INITIAL_CAPACITY);
    StackStack* result = createStackStack(INITIAL_CAPACITY);

    for(int i=1; i < NUM_SENSORS-1; ++i) {
        if(ir_in[i] < IR_MAX) {
            Coordinate coord;
            coord.x = (ir_in[i] + IR_OFFSET) * cos((i * 22.5 - 90) * 3.141592 / 180);
            coord.y = (ir_in[i] + IR_OFFSET) * sin((i * 22.5 - 90) * 3.141592 / 180);
            stackpush(group, &coord);
        }
        else if(stacksize(group) > 0) {
            stackstackpush(groups, group);
            stackclear(group);
        }
    }
    if(stacksize(group) > 0) {
        stackstackpush(groups, group);
        stackclear(group);
    }
    for(int i=0; i<stackstacksize(groups); ++i) {
        StackStack* tmp = runSplitAndMerge(&(groups->stack_data[i]));
        for(int i=0; i<stackstacksize(tmp); ++i) {
            stackstackpush(result, &(tmp->stack_data[i]));
        }
        destroyStackStack(tmp);
    }
    destroyStack(group);
    destroyStackStack(groups);
    return result;
}
#endif