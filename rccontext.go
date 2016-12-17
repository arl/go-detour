package detour

/// Recast log categories.
/// @see rcContext
type rcLogCategory int

const (
	RC_LOG_PROGRESS rcLogCategory = 1 + iota ///< A progress log entry.
	RC_LOG_WARNING                           ///< A warning log entry.
	RC_LOG_ERROR                             ///< An error log entry.
)

/// Provides an interface for optional logging and performance tracking of the Recast
/// build process.
/// @ingroup recast
type rcContexter interface {
	/// clears all log entries
	doResetLog()

	/// Logs a message.
	///  @param[in]		category	The category of the message.
	///  @param[in]		msg			The formatted message.
	///  @param[in]		len			The length of the formatted message.
	doLog(category rcLogCategory, format string, args ...interface{})

	/// Clears all timers. (Resets all to unused.)
	doResetTimers()

	/// Starts the specified performance timer.
	///  @param[in]		label	The category of timer.
	doStartTimer(label rcTimerLabel)

	/// Stops the specified performance timer.
	///  @param[in]		label	The category of the timer.
	doStopTimer(label rcTimerLabel)

	/// Returns the total accumulated time of the specified performance timer.
	///  @param[in]		label	The category of the timer.
	///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	doGetAccumulatedTime(label rcTimerLabel) int
}

type rcContext struct {
	/// True if logging is enabled.
	m_logEnabled bool

	/// True if the performance timers are enabled.
	m_timerEnabled bool

	rcContexter
}

/// Contructor.
///  @param[in]		state	TRUE if the logging and performance timers should be enabled.  [Default: true]
func newRcContext(state bool, ctxer rcContexter) *rcContext {
	return &rcContext{
		m_logEnabled:   state,
		m_timerEnabled: state,
		rcContexter:    ctxer,
	}
}

/// Enables or disables logging.
///  @param[in]		state	TRUE if logging should be enabled.
func (ctx *rcContext) enableLog(state bool) {
	ctx.m_logEnabled = state
}

/// Clears all log entries.
func (ctx *rcContext) resetLog() {
	if ctx.m_logEnabled {
		ctx.doResetLog()
	}
}

/// Logs a message.
///  @param[in]		format		The message.

/// @class rcContext
/// @par
///
/// This class does not provide logging or timer functionality on its
/// own.  Both must be provided by a concrete implementation
/// by overriding the protected member functions.  Also, this class does not
/// provide an interface for extracting log messages. (Only adding them.)
/// So concrete implementations must provide one.
///
/// If no logging or timers are required, just pass an instance of this
/// class through the Recast build process.
///

/// @par
///
/// Example:
/// @code
/// // Where ctx is an instance of rcContext and filepath is a char array.
/// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
/// @endcode
func (ctx *rcContext) log(category rcLogCategory, format string, v ...interface{}) {
	if !ctx.m_logEnabled {
		return
	}
	ctx.doLog(category, format, v...)
}

func (ctx *rcContext) Progressf(format string, v ...interface{}) {
	ctx.log(RC_LOG_PROGRESS, format, v...)
}
func (ctx *rcContext) Warningf(format string, v ...interface{}) {
	ctx.log(RC_LOG_WARNING, format, v...)
}
func (ctx *rcContext) Errorf(format string, v ...interface{}) {
	ctx.log(RC_LOG_ERROR, format, v...)
}

/// Enables or disables the performance timers.
///  @param[in]		state	TRUE if timers should be enabled.
func (ctx *rcContext) enableTimer(state bool) {
	ctx.m_timerEnabled = state
}

/// Clears all peformance timers. (Resets all to unused.)
func (ctx *rcContext) resetTimers() {
	if ctx.m_timerEnabled {
		ctx.doResetTimers()
	}
}

/// Starts the specified performance timer.
///  @param	label	The category of the timer.
func (ctx *rcContext) startTimer(label rcTimerLabel) {
	if ctx.m_timerEnabled {
		ctx.doStartTimer(label)
	}
}

/// Stops the specified performance timer.
///  @param	label	The category of the timer.
func (ctx *rcContext) stopTimer(label rcTimerLabel) {
	if ctx.m_timerEnabled {
		ctx.doStopTimer(label)
	}
}

/// Returns the total accumulated time of the specified performance timer.
///  @param	label	The category of the timer.
///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
func (ctx *rcContext) getAccumulatedTime(label rcTimerLabel) int {

	if ctx.m_timerEnabled {
		return ctx.doGetAccumulatedTime(label)
	} else {
		return -1
	}
}
