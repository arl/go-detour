package recast

import (
	"fmt"
	"time"
)

const maxMessages = 1000

// BuildContext if the build context for recast.
type BuildContext struct {
	startTime [RC_MAX_TIMERS]time.Time
	accTime   [RC_MAX_TIMERS]time.Duration

	messages    [maxMessages]string
	numMessages int
	textPool    string
}

/// Dumps the log to stdout.
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

func (ctx *BuildContext) doResetLog() {
	ctx.numMessages = 0
}

func (ctx *BuildContext) doLog(category LogCategory, msg string) {
	if ctx.numMessages >= maxMessages {
		return
	}
	// Store message
	switch category {
	case RC_LOG_PROGRESS:
		ctx.messages[ctx.numMessages] = "PROG " + msg
	case RC_LOG_WARNING:
		ctx.messages[ctx.numMessages] = "WARN " + msg
	case RC_LOG_ERROR:
		ctx.messages[ctx.numMessages] = "ERR " + msg
	}
	ctx.numMessages++
}

func (ctx *BuildContext) doResetTimers() {
	for i := 0; i < RC_MAX_TIMERS; i++ {
		ctx.accTime[i] = time.Duration(0)
	}
}

func (ctx *BuildContext) doStartTimer(label TimerLabel) {
	ctx.startTime[label] = time.Now()
}

func (ctx *BuildContext) doStopTimer(label TimerLabel) {
	deltaTime := time.Now().Sub(ctx.startTime[label])
	if ctx.accTime[label] == 0 {
		ctx.accTime[label] = deltaTime
	} else {
		ctx.accTime[label] += deltaTime
	}
}

func (ctx *BuildContext) doGetAccumulatedTime(label TimerLabel) time.Duration {
	return ctx.accTime[label]
}
