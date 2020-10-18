#pragma once
#include <memory>

template <class T>
class MovingAverage
{
private:
	unsigned int length;
	int index;
	T * window;
	T current_average;
	bool current_average_is_valid;
	void push(T newValue)
	{
		int next_index = index + 1 >= length ? 0 : index + 1;
		window[index] = newValue;
		index = next_index;
		current_average_is_valid = false;
	}
	void fill(T& initial_values)
	{
		for (unsigned int i = 0; i < length; i++)
		{
			window[i] = initial_values;
		}
		index = 0;
		current_average = initial_values;
		current_average_is_valid = true;
	}
	void add(T newValue)
	{
		if(index == -1){
			fill(newValue);
		} else {
			push(newValue);
		}
	}
public:
	MovingAverage(){
		length = 0;
		index = -1;
		current_average_is_valid = false;
	};
	MovingAverage(const MovingAverage&) = delete; // Disable copying the object
	MovingAverage(unsigned int size) : length(size)
	{
		window = new T[size];
		current_average_is_valid = false;
	};
	MovingAverage(unsigned int size, T& initial_values) : length(size)
	{
		index = 0;
		window = new T[size];
		for (unsigned int i = 0; i < length; i++)
		{
			window[i] = initial_values;
		}

		current_average = initial_values;
		current_average_is_valid = true;
	};
	~MovingAverage()
	{
		delete[] window;
	}
	void init(unsigned int size){
		if(length != 0) delete[] window;
		index = -1;
		window = new T[size];
		length = size;
		current_average_is_valid = false;
	}
	void operator+=(T& newValue)
	{
		add(newValue);
	}
	// For other types
	void operator+=(const T& newValue_raw) // Only for handeling const
	{
		T newValue = newValue_raw;
		add(newValue);
	}
	void operator+=(std::shared_ptr<T> newValue_raw) // Only for handeling shared pointers
	{
		T newValue = * newValue_raw;
		add(newValue);
	}

	T getAverage()
	{
		if(current_average_is_valid){
			return current_average;
		} else {
			T result = window[0];
			for (int i = 1; i < length; i++)
			{
				result += window[i];
			}
			current_average = result / static_cast<double>(length);
			current_average_is_valid = true;
			return current_average;
		}
	}
};