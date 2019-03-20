/*========================= Gait Scheduler ============================*/
/*
 *
 */

#include "Controllers/GaitScheduler.h"


/*=========================== Gait Data ===============================*/
/*
 *
 */
template <typename T>
void GaitData<T>::zero() {

	// Stop any gait transitions
	_nextGait = _currentGait;

	// Enable flag for each foot 
	gaitEnabled = Eigen::Vector4i::Zero();	// enable gaint controlled legs

	// Gait descriptors
	periodTime = Vec4<T>::Zero();		// overall gait period time
	switchingPhase = Vec4<T>::Zero();	// phase to switch to swing

	// Time based descriptors
	timeStance = Vec4<T>::Zero();			// total stance time
	timeSwing = Vec4<T>::Zero();			// total swing time
	timeStanceRemaining = Vec4<T>::Zero();	// stance time remaining
	timeSwingRemaining = Vec4<T>::Zero();	// swing time remaining

	// Phase based descriptors
	phaseVariable = Vec4<T>::Zero();	// overall gait phase for each foot
	phaseOffset = Vec4<T>::Zero();		// nominal gait phase offsets
	phaseScale = Vec4<T>::Zero();		// phase scale relative to variable
	phaseStance = Vec4<T>::Zero();		// stance subphase
	phaseSwing = Vec4<T>::Zero();		// swing subphase

	// Scheduled contact states
	contactStateScheduled = Eigen::Vector4i::Zero();	// contact state of the foot
	contactStatePrev = Eigen::Vector4i::Zero();			// previous contact state of the foot

	// Position of the feet in the world frame at takeoff time
	posFootTakeoffWorld = Mat34<T>::Zero();
}

template struct GaitData<double>;
template struct GaitData<float>;


/*========================= Gait Scheduler ============================*/

/*
 * Initialize the gait data 
 */
template <typename T>
void GaitScheduler<T>::initialize() {
	std::cout << "[GAIT] Initialize Gait Scheduler" << std::endl;

	// Start the 
	gaitData._currentGait = GaitType::TROT;
	gaitData._nextGait = GaitType::TROT;
	gaitData.phaseVariable << 0.0, 0.5, 0.5, 0.0;

	// Create the gait from the nominal initial 
	createGait();
}


/*
 * Executes the Gait Schedule step to calculate values for the defining
 * gait parameters.
 */
template <typename T>
void GaitScheduler<T>::step() {

	// Create a new gait structure if a new gait has been requested
	if (gaitData._currentGait != gaitData._nextGait) {
		printf("[GAIT] Transitioning gait\n");
		createGait();
		gaitData._currentGait = gaitData._nextGait;
	}

	// Iterate over the feet
	for (int foot = 0; foot < 4; foot++) {
		if (gaitData.gaitEnabled(foot) == 1) {
			// Monotonic time based phase incrementation
			dphase = gaitData.phaseScale(foot) * (dt / gaitData.periodTime(foot));

			// Find each foot's current phase
			gaitData.phaseVariable(foot) = fmod((gaitData.phaseVariable(foot) + dphase), 1);

			// Check the current contact state
			if (gaitData.phaseVariable(foot) <= gaitData.switchingPhase(foot)) {
				// Foot is scheduled to be in contact
				gaitData.contactStateScheduled(foot) = 1;

				// Stance subphase calculation
				gaitData.phaseStance(foot) = gaitData.phaseVariable(foot) / gaitData.switchingPhase(foot);

				// Swing phase has not started since foot is in stance
				gaitData.phaseSwing(foot) = 0.0;

				// Calculate the remaining time in stance
				gaitData.timeStanceRemaining(foot) = gaitData.periodTime(foot) * (gaitData.switchingPhase(foot) - gaitData.phaseVariable(foot));

				// Foot is in stance, no swing time remaining
				gaitData.timeSwingRemaining(foot) = 0.0;

			} else {
				// Foot is not scheduled to be in contact
				gaitData.contactStateScheduled(foot) = 0;

				// Stance phase has completed since foot is in swing
				gaitData.phaseStance(foot) = 1.0;

				// Swing subphase calculation
				gaitData.phaseSwing(foot) = (gaitData.phaseVariable(foot) - gaitData.switchingPhase(foot)) / (1.0 - gaitData.switchingPhase(foot));

				// Foot is in swing, no stance time remaining
				gaitData.timeStanceRemaining(foot) = 0.0;

				// Calculate the remaining time in swing
				gaitData.timeSwingRemaining(foot) = gaitData.periodTime(foot) * (1 - gaitData.phaseVariable(foot));
			}

		} else {
			// Leg is not enabled
			gaitData.phaseVariable(foot) = 0.0;

			// Foot is not scheduled to be in contact
			gaitData.contactStateScheduled(foot) = 0;

		}

		// Set the previous contact state for the next timestep
		gaitData.contactStatePrev(foot) = gaitData.contactStateScheduled(foot);
		

		//std::cout << gaitData.contactStateScheduled(foot) << ", " << gaitData.gaitName << " | ";
	}
	//std::cout << std::endl;
	printGaitInfo();
}


/*
 * Creates the gait structure from the important defining parameters of each gait
 */
template <typename T>
void GaitScheduler<T>::createGait() {
	// Case structure gets the appropriate parameters
	switch (gaitData._nextGait) {
	case GaitType::STAND :
		gaitData.gaitName = "STAND";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 10.0, 10.0, 10.0, 10.0;
		gaitData.switchingPhase << 1.0, 1.0, 1.0, 1.0;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::STAND_CYCLE :
		gaitData.gaitName = "STAND_CYCLE";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 10.0, 10.0, 10.0, 10.0;
		gaitData.switchingPhase << 1.0, 1.0, 1.0, 1.0;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::STATIC_WALK :
		gaitData.gaitName = "STATIC_WALK";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 1.25, 1.25, 1.25, 1.25;
		gaitData.switchingPhase << 0.8, 0.8, 0.8, 0.8;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::AMBLE :
		gaitData.gaitName = "AMBLE";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 1.0, 1.0, 1.0, 1.0;
		gaitData.switchingPhase << 0.8, 0.8, 0.8, 0.8;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::TROT_WALK :
		gaitData.gaitName = "TROT_WALK";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.6, 0.6, 0.6, 0.6;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::TROT :
		gaitData.gaitName = "TROT";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.5, 0.5, 0.5, 0.5;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::TROT_RUN :
		gaitData.gaitName = "TROT_RUN";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.4, 0.4, 0.4, 0.4;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::PACE :
		gaitData.gaitName = "PACE";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.5, 0.5, 0.5, 0.5;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::BOUND :
		gaitData.gaitName = "BOUND";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.5, 0.5, 0.5, 0.5;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::ROTARY_GALLOP :
		gaitData.gaitName = "ROTARY_GALLOP";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.2, 0.2, 0.2, 0.2;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::TRAVERSE_GALLOP :
		gaitData.gaitName = "TRAVERSE_GALLOP";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.2, 0.2, 0.2, 0.2;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::PRONK :
		gaitData.gaitName = "PRONK";
		gaitData.gaitEnabled << 1, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.5, 0.5, 0.5, 0.5;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::THREE_FOOT :
		gaitData.gaitName = "THREE_FOOT";
		gaitData.gaitEnabled << 0, 1, 1, 1;
		gaitData.periodTime << 0.5, 0.5, 0.5, 0.5;
		gaitData.switchingPhase << 0.5, 0.5, 0.5, 0.5;
		gaitData.phaseScale << 1.0, 1.0, 1.0, 1.0;
		break;

	case GaitType::CUSTOM :
		gaitData.gaitName = "CUSTOM";
		// TODO: get custom gait parameters from operator GUI
		break;

	}

	// Set the gait parameters for each foot
	for (int foot = 0; foot < 4; foot++) {
		if (gaitData.gaitEnabled(foot) == 1) {
			// Find the total stance time over the gait cycle
			gaitData.timeStance(foot) = gaitData.periodTime(foot) * gaitData.switchingPhase(foot);

			// Find the total swing time over the gait cycle
			gaitData.timeSwing(foot) = gaitData.periodTime(foot) * (1.0 - gaitData.switchingPhase(foot));
		
		} else {
			// Foot is never in stance
			gaitData.timeStance(foot) = 0.0;

			// Foot is always in "swing"
			gaitData.timeSwing(foot) = gaitData.periodTime(foot);
		}
	}
}


/*
 * Creates the gait structure from the important defining parameters of each gait
 */
template <typename T>
void GaitScheduler<T>::printGaitInfo() {
	std::cout << "Gait Type: " << gaitData.gaitName << "\n";
	std::cout << "---------------------------------------------------------\n";
	std::cout << "Enabled: " << gaitData.gaitEnabled(0) << " " << gaitData.gaitEnabled(1) << " " << gaitData.gaitEnabled(2) << " " << gaitData.gaitEnabled(3) << "\n";
	std::cout << "Period Time: " << gaitData.periodTime(0) << "s " << gaitData.periodTime(1) << "s " << gaitData.periodTime(2) << "s " << gaitData.periodTime(3) << "s\n";
	std::cout << "---------------------------------------------------------\n";
	std::cout << "Contact State: " << gaitData.contactStateScheduled(0) << " | " << gaitData.contactStateScheduled(1) << " | " << gaitData.contactStateScheduled(2) << " | " << gaitData.contactStateScheduled(3) << "\n";
	std::cout << "Phase Variable: " << gaitData.phaseVariable(0) << " | " << gaitData.phaseVariable(1) << " | " << gaitData.phaseVariable(2) << " | " << gaitData.phaseVariable(3) << "\n";
	std::cout << "Stance Time Remaining: " << gaitData.timeStanceRemaining(0) << "s | " << gaitData.timeStanceRemaining(1) << "s | " << gaitData.timeStanceRemaining(2) << "s | " << gaitData.timeStanceRemaining(3) << "s\n";
	std::cout << "Swing Time Remaining: " << gaitData.timeSwingRemaining(0) << "s | " << gaitData.timeSwingRemaining(1) << "s | " << gaitData.timeSwingRemaining(2) << "s | " << gaitData.timeSwingRemaining(3) << "s\n";
	std::cout << std::endl;
}


template class GaitScheduler<double>;
template class GaitScheduler<float>;