package recast

import (
	"fmt"
	"time"
)

// Recast log categories.
// see BuildContext
type logCategory int

const (
	logProgress logCategory = 1 + iota // A progress log entry.
	logWarning                         // A warning log entry.
	logError                           // An error log entry.
)

const maxMessages = 1000

// BuildContext provides an interface for optional logging and performance
// tracking of the recast build process.
type BuildContext struct {
	startTime [maxTimers]time.Time
	accTime   [maxTimers]time.Duration

	messages    [maxMessages]string
	numMessages int
	textPool    string

	// True if logging is enabled.
	logEnabled bool

	// True if the performance timers are enabled.
	timerEnabled bool
}

// NewBuildContext returns an initialized buildcontext where state indicated if
// logging and performance tracking are enabled.
func NewBuildContext(state bool) *BuildContext {
	return &BuildContext{
		logEnabled:   state,
		timerEnabled: state,
	}
}

// EnableLog enables or disables logging.
func (ctx *BuildContext) EnableLog(state bool) {
	ctx.logEnabled = state
}

// EnableTimer enables or disables the performance timers.
func (ctx *BuildContext) EnableTimer(state bool) {
	ctx.timerEnabled = state
}

// ResetLog clears all log entries.
func (ctx *BuildContext) ResetLog() {
	if ctx.logEnabled {
		ctx.numMessages = 0
	}
}

// ResetTimers clears all peformance timers. (Resets all to unused.)
func (ctx *BuildContext) ResetTimers() {
	if ctx.timerEnabled {
		for i := 0; i < maxTimers; i++ {
			ctx.accTime[i] = time.Duration(0)
		}
	}
}

// Progressf writes a new log entry in the 'progress' category.
//
// The format string and arguments are forwarded to fmt.Sprintf and thus accepts
// the same format specifiers.
func (ctx *BuildContext) Progressf(format string, v ...interface{}) {
	ctx.log(logProgress, format, v...)
}

// Warningf writes a new log entry in the 'warning' category.
//
// The format string and arguments are forwarded to fmt.Sprintf and thus accepts
// the same format specifiers.
func (ctx *BuildContext) Warningf(format string, v ...interface{}) {
	ctx.log(logWarning, format, v...)
}

// Errorf writes a new log entry in the 'error' category.
//
// The format string and arguments are forwarded to fmt.Sprintf and thus accepts
// the same format specifiers.
func (ctx *BuildContext) Errorf(format string, v ...interface{}) {
	ctx.log(logError, format, v...)
}

// log writes a new log entry in the specified category.
//
// The format string and arguments are forwarded to fmt.Sprintf and thus accepts
// the same format specifiers.
func (ctx *BuildContext) log(category logCategory, format string, v ...interface{}) {
	if ctx.logEnabled && ctx.numMessages < maxMessages {
		// Store message
		switch category {
		case logProgress:
			ctx.messages[ctx.numMessages] = "PROG " + fmt.Sprintf(format, v...)
		case logWarning:
			ctx.messages[ctx.numMessages] = "WARN " + fmt.Sprintf(format, v...)
		case logError:
			ctx.messages[ctx.numMessages] = "ERR " + fmt.Sprintf(format, v...)
		}
		ctx.numMessages++
	}
}

// DumpLog dumps all the log entries to stdout, preceded by a message.
//
// The format string and arguments are forwarded to fmt.Sprintf and thus accepts
// the same format specifiers.
func (ctx *BuildContext) DumpLog(format string, args ...interface{}) {
	// Print header.
	fmt.Printf(format+"\n", args...)

	// Print messages
	for i := 0; i < ctx.numMessages; i++ {
		msg := ctx.messages[i]
		fmt.Println(msg)
	}
}

// LogCount returns the current number of log entries.
func (ctx *BuildContext) LogCount() int {
	return ctx.numMessages
}

// LogText returns the log entry at index i.
func (ctx *BuildContext) LogText(i int32) string {
	return ctx.messages[i]
}

// StartTimer starts the specified performance timer.
func (ctx *BuildContext) StartTimer(label TimerLabel) {
	if ctx.timerEnabled {
		ctx.startTime[label] = time.Now()
	}
}

// StopTimer stops the specified performance timer.
func (ctx *BuildContext) StopTimer(label TimerLabel) {
	if ctx.timerEnabled {
		deltaTime := time.Now().Sub(ctx.startTime[label])
		if ctx.accTime[label] == 0 {
			ctx.accTime[label] = deltaTime
		} else {
			ctx.accTime[label] += deltaTime
		}
	}
}

// AccumulatedTime returns the total accumulated time of the specified
// performance timer.
//
// Returns the accumulated time of the timer, or -1 if timers are disabled or
// the timer has never been started.
func (ctx *BuildContext) AccumulatedTime(label TimerLabel) time.Duration {
	if ctx.timerEnabled {
		return ctx.accTime[label]
	}
	return time.Duration(0)
}
