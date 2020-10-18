#include "TrackedGate.h"

TrackedGate::TrackedGate(int window_size_gate, int window_size_persistance, Gate& gate){
    gates_moving_avg.init(window_size_gate);
    gates_moving_avg += gate;
    persistance.init(window_size_persistance);
    persistance += 0.0;
    persistance += 1.0;
    active_state = false;
    touched = true;
};

bool TrackedGate::merge(Gate& new_gate)
{
    gates_moving_avg += new_gate;
    persistance += 1.0;
    touched = true;
    return true;
};

float TrackedGate::distanceToSqr(Gate& newGate){
    Gate temp = gates_moving_avg.getAverage();
    float x = (temp.center.x - newGate.center.x);
    float y = (temp.center.y - newGate.center.y);
    float z = (temp.center.z - newGate.center.z);
    return x*x + y*y + z*z;
};
void TrackedGate::clearTouched()
{
    touched = false;
};
void TrackedGate::updatePersistance()
{
    if(!touched) persistance += 0.0;
    clearTouched();
}
Gate TrackedGate::getAvgGate()
{
    return gates_moving_avg.getAverage();
};

float TrackedGate::getPersistance()
{
    return persistance.getAverage();
};
bool TrackedGate::getActiveState()
{
    return active_state;
};
void TrackedGate::setActiveState(bool state)
{
    active_state = state;
};