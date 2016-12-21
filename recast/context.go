package recast

import (
	"fmt"
	"time"
)

/// Recast log categories.
/// @see Context
type LogCategory int

const (
	RC_LOG_PROGRESS LogCategory = 1 + iota ///< A progress log entry.
	RC_LOG_WARNING                         ///< A warning log entry.
	RC_LOG_ERROR                           ///< An error log entry.
)

/// Provides an interface for optional logging and performance tracking of the Recast
/// build process.
/// @ingroup recast
type Contexter interface {
	/// clears all log entries
	doResetLog()

	/// Logs a message.
	///  @param[in]		category	The category of the message.
	///  @param[in]		msg			The formatted message.
	///  @param[in]		len			The length of the formatted message.
	doLog(category LogCategory, msg string)

	/// Clears all timers. (Resets all to unused.)
	doResetTimers()

	/// Starts the specified performance timer.
	///  @param[in]		label	The category of timer.
	doStartTimer(label TimerLabel)

	/// Stops the specified performance timer.
	///  @param[in]		label	The category of the timer.
	doStopTimer(label TimerLabel)

	/// Returns the total accumulated time of the specified performance timer.
	///  @param[in]		label	The category of the timer.
	///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	doGetAccumulatedTime(label TimerLabel) time.Duration
}

type Context struct {
	/// True if logging is enabled.
	m_logEnabled bool

	/// True if the performance timers are enabled.
	m_timerEnabled bool

	Contexter
}

/// Contructor.
///  @param[in]		state	TRUE if the logging and performance timers should be enabled.  [Default: true]
func NewContext(state bool, ctxer Contexter) *Context {
	return &Context{
		m_logEnabled:   state,
		m_timerEnabled: state,
		Contexter:      ctxer,
	}
}

/// Enables or disables logging.
///  @param[in]		state	TRUE if logging should be enabled.
func (ctx *Context) enableLog(state bool) {
	ctx.m_logEnabled = state
}

/// Clears all log entries.
func (ctx *Context) resetLog() {
	if ctx.m_logEnabled {
		ctx.doResetLog()
	}
}

/// Logs a message.
///  @param[in]		format		The message.

/// @class Context
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
/// // Where ctx is an instance of Context and filepath is a char array.
/// ctx->log(_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
/// @endcode
func (ctx *Context) Log(category LogCategory, format string, v ...interface{}) {
	if !ctx.m_logEnabled {
		return
	}
	ctx.doLog(category, fmt.Sprintf(format, v...))
}

func (ctx *Context) Progressf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_PROGRESS, format, v...)
}
func (ctx *Context) Warningf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_WARNING, format, v...)
}
func (ctx *Context) Errorf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_ERROR, format, v...)
}

/// Enables or disables the performance timers.
///  @param[in]		state	TRUE if timers should be enabled.
func (ctx *Context) enableTimer(state bool) {
	ctx.m_timerEnabled = state
}

/// Clears all peformance timers. (Resets all to unused.)
func (ctx *Context) resetTimers() {
	if ctx.m_timerEnabled {
		ctx.doResetTimers()
	}
}

/// Starts the specified performance timer.
///  @param	label	The category of the timer.
func (ctx *Context) startTimer(label TimerLabel) {
	if ctx.m_timerEnabled {
		ctx.doStartTimer(label)
	}
}

/// Stops the specified performance timer.
///  @param	label	The category of the timer.
func (ctx *Context) stopTimer(label TimerLabel) {
	if ctx.m_timerEnabled {
		ctx.doStopTimer(label)
	}
}

/// Returns the total accumulated time of the specified performance timer.
///  @param	label	The category of the timer.
///  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
func (ctx *Context) AccumulatedTime(label TimerLabel) time.Duration {

	if ctx.m_timerEnabled {
		return ctx.doGetAccumulatedTime(label)
	} else {
		return time.Duration(0)
	}
}
