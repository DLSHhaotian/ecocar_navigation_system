#include <stdexcept>
#include "my_queue.h"

// Constructor
MyQueue::MyQueue(int maxQueueSize) {
    if(maxQueueSize <= 0) throw std::invalid_argument("MyQueue received negative or zero value");
    MyQueue::maxQueueSize = (unsigned int)maxQueueSize;
    steeringEncoderQueue = new float[maxQueueSize]();
    steeringEncoderQueueIndex = 0;
    queueSize = 0;
    sum = 0;
}

// Get the average of all elements in the queue
float MyQueue::getAverage() {
    if(queueSize == 0) return 0;
    return (float)(sum / queueSize);
}

// Get the last element in the Queue
float MyQueue::getLast() {
    return steeringEncoderQueue[(steeringEncoderQueueIndex + 1) % maxQueueSize];
}

// Push a value into the Queue
void MyQueue::add(float value) {
    if(queueSize < maxQueueSize) queueSize++;
    sum = sum - steeringEncoderQueue[steeringEncoderQueueIndex] + value;
    steeringEncoderQueue[steeringEncoderQueueIndex] = value;
    steeringEncoderQueueIndex = (steeringEncoderQueueIndex + 1) % maxQueueSize;
}

// Reset all values in the queue to the set value
void MyQueue::reset(float resetValue) {
    for(int i = 0; i < maxQueueSize; i++) {
        add(resetValue);
    }
}

// Empty the queue
void MyQueue::empty() {
    reset(0);
    queueSize = 0;
}