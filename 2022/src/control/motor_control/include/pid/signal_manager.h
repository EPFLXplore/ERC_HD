/**
 * @file signal_manager.h
 *
 * @date May 4, 2016
 * @author Benjamin Navarro
 * @brief SignalManager include file
 */


#ifndef PID_SIGNAL_MANAGER_H
#define PID_SIGNAL_MANAGER_H

#include <map>
#include <list>
#include <signal.h>
#include <functional>

namespace pid {

/**
 * @class SignalManager SignalManager.h
 * @brief A UNIX signal manager. Can be used to register multiple callback functions on the same signal.
 */
class SignalManager {
public:
	typedef std::function<void(int)> callback_type;

	/**
	 * @brief Default constructor, does nothing.
	 */
	SignalManager() = default;

	/**
	 * @brief Register a new callback function.
	 * @param signal Signal number to catch. May be a field of SignalManager#Signals.
	 * @param name A user-defined name to identify the callback function.
	 * @param callback A pointer to callback function.
	 * @return true if successful, false otherwise (a callback function with the same name is already registered on that signal)
	 */
	static bool registerCallback(int signal, const std::string& name, callback_type callback);

	/**
	 * @brief Unregister a callback function
	 * @param signal The signal to unregister the callback from
	 * @param name The name of the callback (as given in SignalManager#registerCallback)
	 * @return true if successful, false otherwise (no callbacks for this \a signal or no callback named \a name is registered to this \a signal.
	 */
	static bool unregisterCallback(int signal, const std::string& name);

	/**
	 * @brief Redefinition of signals' standard name for clarity
	 */
	enum Signals {
		Hangup 					= SIGHUP,   //!< SIGHUP
		Interrupt 				= SIGINT,   //!< SIGINT
		Quit 					= SIGQUIT,  //!< SIGQUIT
		IllegalInstruction		= SIGILL,	//!< SIGILL
		TraceTrap 				= SIGTRAP,  //!< SIGTRAP
		Abort 					= SIGABRT,  //!< SIGABRT
		IOTTrap					= SIGIOT,   //!< SIGIOT
		BusError				= SIGBUS,   //!< SIGBUS
		FLoatingPointException	= SIGFPE,	//!< SIGFPE
		KillSignal 				= SIGKILL,  //!< SIGKILL
		UserDefined1			= SIGUSR1,  //!< SIGUSR1
		SegmentationViolation	= SIGSEGV,	//!< SIGSEGV
		UserDefined2			= SIGUSR2,  //!< SIGUSR2
		BrokenPipe				= SIGPIPE,  //!< SIGPIPE
		AlarmClock				= SIGALRM,  //!< SIGALRM
		Termination				= SIGTERM,  //!< SIGTERM
		StackFault				= SIGSTKFLT,//!< SIGSTKFLT
		ChildStatusHasChanged	= SIGCHLD,	//!< SIGCHLD
		Continue 				= SIGCONT,  //!< SIGCONT
		Stop					= SIGSTOP,  //!< SIGSTOP
		KeyboardStop			= SIGTSTP,  //!< SIGTSTP
		BackgroundReadFromTTY	= SIGTTIN,	//!< SIGTTIN
		BackgroundWriteToTTY	= SIGTTOU, 	//!< SIGTTOU
		UrgentCondition			= SIGURG,   //!< SIGURG
		CPULimitExceeded		= SIGXCPU,  //!< SIGXCPU
		FileSizeLimitExceeded	= SIGXFSZ,	//!< SIGXFSZ
		VirtualAlarmClock		= SIGVTALRM,//!< SIGVTALRM
		ProfilingAlarmClock		= SIGPROF, 	//!< SIGPROF
		WindowSizeChange		= SIGWINCH, //!< SIGWINCH
		PollageEventOccured		= SIGPOLL, 	//!< SIGPOLL
		IONowPossible			= SIGIO	,   //!< SIGIO
		PowerFailureRestart		= SIGPWR,  	//!< SIGPWR
		BadSystemCall			= SIGSYS,   //!< SIGSYS
	};

private:

	/**
	 * @brief Handles all the incoming registered signals and dispatch them to the appropriate handlers
	 * @param signal The received signal
	 */
	static void general_callback(int signal);
};

} /* namespace pid */

#endif /* PID_SIGNAL_MANAGER_H */
