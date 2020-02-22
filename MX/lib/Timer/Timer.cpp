#include "Timer.h"




// ------------------------------------------------------------
// static class variables need to be reiterated here before use
// ------------------------------------------------------------
bool Timer::PIT_enabled;
bool Timer::PIT_used[];
Timer::ISR Timer::PIT_ISR[];

// ------------------------------------------------------------
// these are the ISRs (Interrupt Service Routines) that get
// called by each PIT timer when it fires. they're defined here
// so that they can auto-clear themselves and so the user can
// specify a custom ISR and reassign it as needed
// ------------------------------------------------------------
#if defined(KINETISK)
void pit0_isr() {
	PIT_TFLG0 = 1;
	Timer::PIT_ISR[0]();
}
void pit1_isr() {
	PIT_TFLG1 = 1;
	Timer::PIT_ISR[1]();
}
void pit2_isr() {
	PIT_TFLG2 = 1;
	Timer::PIT_ISR[2]();
}
void pit3_isr() {
	PIT_TFLG3 = 1;
	Timer::PIT_ISR[3]();
}

#elif defined(KINETISL)
void pit_isr() {
	if (PIT_TFLG0) {PIT_TFLG0 = 1; Timer::PIT_ISR[0]();}
	if (!Timer::PIT_enabled) return;
	if (PIT_TFLG1) {PIT_TFLG1 = 1; Timer::PIT_ISR[1]();}
}
#endif

// ------------------------------------------------------------
// this function inits and starts the timer, using the specified
// function as a callback and the period provided. must be passed
// the name of a function taking no arguments and returning void.
// make sure this function can complete within the time allowed.
// attempts to allocate a timer using available resources,
// returning true on success or false in case of failure.
// period is specified as number of bus cycles
// ------------------------------------------------------------
bool Timer::beginCycles(ISR newISR, uint32_t newValue) {

	// if this interval timer is already running, stop it
	if (status == TIMER_PIT) {
		stop_PIT();
		status = TIMER_OFF;
	}
	// store callback pointer
	myISR = newISR;

	// attempt to allocate this timer
	if (allocate_PIT(newValue))
		status = TIMER_PIT;
	else
		status = TIMER_OFF;

	// check for success and return
	if (status != TIMER_OFF)
		return true;
	return false;

}

// ------------------------------------------------------------
// stop the timer if it's currently running, using its status
// to determine what hardware resources the timer may be using
// ------------------------------------------------------------
void Timer::end() {
	if (status == TIMER_PIT)
		stop_PIT();
	status = TIMER_OFF;
}

// ------------------------------------------------------------
// enables the PIT clock bit, the master PIT reg, and sets flag
// ------------------------------------------------------------
void Timer::enable_PIT() {
	SIM_SCGC6 |= SIM_SCGC6_PIT;
	PIT_MCR = 0;
	PIT_enabled = true;
}

// ------------------------------------------------------------
// disables the master PIT reg, the PIT clock bit, and unsets flag
// ------------------------------------------------------------
void Timer::disable_PIT() {
	PIT_MCR = 1;
	SIM_SCGC6 &= ~SIM_SCGC6_PIT;
	PIT_enabled = false;
}

// ------------------------------------------------------------
// enables the PIT clock if not already enabled, then checks to
// see if any PITs are available for use. if one is available,
// it's initialized and started with the specified value, and
// the function returns true, otherwise it returns false
// ------------------------------------------------------------
bool Timer::allocate_PIT(uint32_t newValue) {

	// enable clock to the PIT module if necessary
	if (!PIT_enabled)
		enable_PIT();

	// check for an available PIT, and if so, start it
	for (uint8_t id = 0; id < NUM_PIT; id++) {
		if (!PIT_used[id]) {
			PIT_id = id;
			start_PIT(newValue);
			PIT_used[id] = true;
			return true;
		}
	}

	// no PIT available
	return false;

}


// ------------------------------------------------------------
// calling this function causes the current countdown cycle of the
// timer to reset, essentially delaying the firing of the callback
// until another full period of the timer's cycle has elapsed.
// ------------------------------------------------------------
void Timer::reset() {
  *PIT_TCTRL = 1;
  *PIT_TCTRL = 3;
  _cv = 0;
}


// ------------------------------------------------------------
// Cette methode est cens�e donner la valeur de cv
//
// ------------------------------------------------------------
uint32_t Timer::readValue(void){
	return *PIT_LDVAL;
}


// ------------------------------------------------------------
// configuters a PIT's registers, function pointer, and enables
// interrupts, effectively starting the timer upon completion
// ------------------------------------------------------------
void Timer::start_PIT(uint32_t newValue) {

	// point to the correct registers
	PIT_LDVAL = &PIT_LDVAL0 + PIT_id * 4;
	PIT_TCTRL = &PIT_TCTRL0 + PIT_id * 4;

	// point to the correct PIT ISR
	PIT_ISR[PIT_id] = myISR;

	// write value to register and enable interrupt
	*PIT_TCTRL = 0;
	*PIT_LDVAL = newValue;
	*PIT_TCTRL = 3;
#if defined(KINETISK)
	IRQ_PIT_CH = IRQ_PIT_CH0 + PIT_id;
	NVIC_SET_PRIORITY(IRQ_PIT_CH, nvic_priority);
	NVIC_ENABLE_IRQ(IRQ_PIT_CH);
#elif defined(KINETISL)
	NVIC_SET_PRIORITY(IRQ_PIT, nvic_priority); // TODO: use the higher of both channels, shared irq
	NVIC_ENABLE_IRQ(IRQ_PIT);
#endif

}



// ------------------------------------------------------------
// stops an active PIT by disabling its interrupt, writing to
// its control register, and freeing up its state for future use.
// also, if no PITs remain in use, disables the core PIT clock
// ------------------------------------------------------------
void Timer::stop_PIT() {

	// disable interrupt and PIT
	*PIT_TCTRL = 0;
#if defined(KINETISK)
	NVIC_DISABLE_IRQ(IRQ_PIT_CH);
#elif defined(KINETISL)
	NVIC_DISABLE_IRQ(IRQ_PIT);
#endif

	// free PIT for future use
	PIT_used[PIT_id] = false;

	// check if we're still using any PIT
	for (uint8_t id = 0; id < NUM_PIT; id++) {
		if (PIT_used[id])
			return;
	}

	// none used, disable PIT clock
	disable_PIT();

}

//Tester si le timer doit mieux etre dans la classe chenillard ou directement dans la classe
//Dans la classe main et le chenillard viendrai se calquer dessus
//A moin qu'il soit directement dans la classe timer
// Le but est de pouvoir lancer simmultann�ement
//2 chenillards sur le meme bloc

/********\
| inc_cv | : Tant que le timer est demarr� => incr�mente le current value jusqu'a la valeur de RC
\********/
void Timer::inc_cv(uint16_t rc) {
		_rc = rc;
	//if (status == true) {
		_cv++;
		if (_cv >= rc) {
			_cv = 0;
		}
	//}
}

/********\
| get_cv | : retourne la valeur de cv
\********/
uint16_t Timer::get_cv(void) {
	return _cv;
}

/********\
| set_cv | : defini la valeur de cv
\********/
void Timer::set_cv(uint16_t cv) {
	_cv = cv;
}


/********\
| get_RC | : Permet de lire al valeur de RC
 \********/
uint16_t Timer::get_rc(void) {
	return _rc;
}
