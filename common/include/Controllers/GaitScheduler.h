#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <cppTypes.h>
#include <iostream>


/*
 *
 */
enum class GaitType {
	STAND,
	STAND_CYCLE,
	STATIC_WALK,
	AMBLE,
	TROT_WALK,
	TROT,
	TROT_RUN,
	PACE,
	BOUND,
	ROTARY_GALLOP,
	TRAVERSE_GALLOP,
	PRONK,
	THREE_FOOT,
	CUSTOM
};


/*
 *
 */
template <typename T>
struct GaitData {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	GaitData() {
		zero();
	}

	// Zero out all of the data
	void zero();

	// The current GaitType 
	GaitType _currentGait; 

	// Next GaitType to transition into
	GaitType _nextGait; 

	std::string gaitName;

	// Enable flag for each foot 
	Eigen::Vector4i gaitEnabled;	// enable gaint controlled legs

	// Gait descriptors
	Vec4<T> periodTime;			// overall gait period time
	Vec4<T> switchingPhase;		// phase to switch to swing

	// Time based descriptors
	Vec4<T> timeStance;				// total stance time
	Vec4<T> timeSwing;				// total swing time
	Vec4<T> timeStanceRemaining;	// stance time remaining
	Vec4<T> timeSwingRemaining;		// swing time remaining

	// Phase based descriptors
	Vec4<T> phaseVariable;	// overall gait phase for each foot
	Vec4<T> phaseOffset;	// nominal gait phase offsets
	Vec4<T> phaseScale;		// phase scale relative to variable
	Vec4<T> phaseStance;	// stance subphase
	Vec4<T> phaseSwing;		// swing subphase

	// Scheduled contact states
	Eigen::Vector4i contactStateScheduled;	// contact state of the foot
	Eigen::Vector4i contactStatePrev;		// previous contact state of the foot

	// Position of the feet in the world frame at takeoff time
	Mat34<T> posFootTakeoffWorld;
};


/*
 *
 */
template <typename T>
class GaitScheduler {
public:
	// Initial
	void initialize();
	void step();
	void createGait();
	void printGaitInfo();

	// Struct containing all of the gait relevant data
	GaitData<T> gaitData;

private:
	// Control loop timestep change
	T dt = 0.001;

	// Phase change at each step
	T dphase;  

};

#endif