package recast

import (
	"fmt"
	"time"
)

// Recast log categories.
// @see Context
type LogCategory int

const (
	RC_LOG_PROGRESS LogCategory = 1 + iota // A progress log entry.
	RC_LOG_WARNING                         // A warning log entry.
	RC_LOG_ERROR                           // An error log entry.
)

const maxMessages = 1000

// BuildContext if the build context for recast.
//
// This class does not provide logging or timer functionality on its
// own.  Both must be provided by a concrete implementation
// by overriding the protected member functions.  Also, this class does not
// provide an interface for extracting log messages. (Only adding them.)
// So concrete implementations must provide one.
//
// If no logging or timers are required, just pass an instance of this
// class through the Recast build process.
type BuildContext struct {
	startTime [RC_MAX_TIMERS]time.Time
	accTime   [RC_MAX_TIMERS]time.Duration

	messages    [maxMessages]string
	numMessages int
	textPool    string

	/// True if logging is enabled.
	m_logEnabled bool

	/// True if the performance timers are enabled.
	m_timerEnabled bool
}

func NewBuildContext(state bool) *BuildContext {
	return &BuildContext{
		m_logEnabled:   state,
		m_timerEnabled: state,
	}
}

// Enables or disables logging.
//  @param[in]		state	TRUE if logging should be enabled.
func (ctx *BuildContext) EnableLog(state bool) {
	ctx.m_logEnabled = state
}

// Enables or disables the performance timers.
//  @param[in]		state	TRUE if timers should be enabled.
func (ctx *BuildContext) EnableTimer(state bool) {
	ctx.m_timerEnabled = state
}

// Clears all log entries.
func (ctx *BuildContext) ResetLog() {
	if ctx.m_logEnabled {
		ctx.numMessages = 0
	}
}

// Clears all peformance timers. (Resets all to unused.)
func (ctx *BuildContext) ResetTimers() {
	if ctx.m_timerEnabled {
		for i := 0; i < RC_MAX_TIMERS; i++ {
			ctx.accTime[i] = time.Duration(0)
		}
	}
}

func (ctx *BuildContext) Progressf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_PROGRESS, format, v...)
}

func (ctx *BuildContext) Warningf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_WARNING, format, v...)
}

func (ctx *BuildContext) Errorf(format string, v ...interface{}) {
	ctx.Log(RC_LOG_ERROR, format, v...)
}

// Logs a message.
//  @param[in]		format		The message.
func (ctx *BuildContext) Log(category LogCategory, format string, v ...interface{}) {
	if ctx.m_logEnabled && ctx.numMessages < maxMessages {
		// Store message
		switch category {
		case RC_LOG_PROGRESS:
			ctx.messages[ctx.numMessages] = "PROG " + fmt.Sprintf(format, v...)
		case RC_LOG_WARNING:
			ctx.messages[ctx.numMessages] = "WARN " + fmt.Sprintf(format, v...)
		case RC_LOG_ERROR:
			ctx.messages[ctx.numMessages] = "ERR " + fmt.Sprintf(format, v...)
		}
		ctx.numMessages++
	}
}

// Dumps the log to stdout.
func (ctx *BuildContext) DumpLog(format string, args ...interface{}) {

	// Print header.
	fmt.Printf(format+"\n", args...)

	// Print messages
	for i := 0; i < ctx.numMessages; i++ {
		msg := ctx.messages[i]
		fmt.Println(msg)
	}
}

func (ctx *BuildContext) LogCount() int {
	return ctx.numMessages
}

/// Returns log message text.
func (ctx *BuildContext) LogText(i int32) string {
	return ctx.messages[i]
}

// Starts the specified performance timer.
//  @param	label	The category of the timer.
func (ctx *BuildContext) StartTimer(label TimerLabel) {
	if ctx.m_timerEnabled {
		ctx.startTime[label] = time.Now()
	}
}

// Stops the specified performance timer.
//  @param	label	The category of the timer.
func (ctx *BuildContext) StopTimer(label TimerLabel) {
	if ctx.m_timerEnabled {
		deltaTime := time.Now().Sub(ctx.startTime[label])
		if ctx.accTime[label] == 0 {
			ctx.accTime[label] = deltaTime
		} else {
			ctx.accTime[label] += deltaTime
		}
	}
}

// Returns the total accumulated time of the specified performance timer.
//  @param	label	The category of the timer.
//  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
func (ctx *BuildContext) AccumulatedTime(label TimerLabel) time.Duration {
	if ctx.m_timerEnabled {
		return ctx.accTime[label]
	} else {
		return time.Duration(0)
	}
}
