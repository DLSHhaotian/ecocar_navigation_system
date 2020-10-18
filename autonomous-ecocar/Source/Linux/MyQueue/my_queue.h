#ifndef MY_QUEUE_h
#define MY_QUEUE_h

class MyQueue {
    private:
        unsigned int maxQueueSize;
        unsigned int queueSize;
        double sum;
        float* steeringEncoderQueue;
        unsigned int steeringEncoderQueueIndex;
    public:
        MyQueue(int queueSize);
        float getAverage();
        float getLast();
        void add(float value);
        void reset(float value = 0);
        void empty();
};

#endif // MY_QUEUE_h