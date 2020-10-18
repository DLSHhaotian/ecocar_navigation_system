#pragma once
#include "Gate.h"
#include "MovingAverage.h"
#include <iostream> // Debug

class TrackedGate
{
  private:
    MovingAverage<Gate> gates_moving_avg;
    MovingAverage<float> persistance;
    bool active_state; // For hysteresis
    bool touched;
    
  public:
    typedef std::shared_ptr<TrackedGate> ptr;
    TrackedGate(int window_size_gate, int window_size_persistance, Gate& gate);
    bool merge(Gate& new_gate);
    float distanceToSqr(Gate& newGate);
    void clearTouched();
    void updatePersistance();
    Gate getAvgGate();
    float getPersistance();
    bool getActiveState();
    void setActiveState(bool state);
};