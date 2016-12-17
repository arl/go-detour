package detour

import (
	"fmt"
	"time"
)

const (
	MAX_MESSAGES   = 1000
	TEXT_POOL_SIZE = 8000
)

/// Recast build context.
type BuildContext struct {
	m_startTime [RC_MAX_TIMERS]time.Time
	m_accTime   [RC_MAX_TIMERS]time.Duration

	m_messages     [MAX_MESSAGES]string
	m_messageCount int
	m_textPool     string
	//int m_textPoolSize;
}

/// Dumps the log to stdout.
func (ctx *BuildContext) dumpLog(format string, args ...interface{}) {

	// Print header.
	fmt.Printf(format+"\n", args...)

	// Print messages
	//TAB_STOPS := [4]int{28, 36, 44, 52}
	for i := 0; i < ctx.m_messageCount; i++ {
		msg := ctx.m_messages[i]
		fmt.Println(msg)
	}
}

func (ctx *BuildContext) LogCount() int {
	return ctx.m_messageCount
}

/// Returns log message text.
func (ctx *BuildContext) LogText(i int32) string {
	return ctx.m_messages[i]
}

func (ctx *BuildContext) doResetLog() {
	ctx.m_messageCount = 0
}

func (ctx *BuildContext) doLog(category rcLogCategory, msg string) {
	if ctx.m_messageCount >= MAX_MESSAGES {
		return
	}
	// Store message
	switch category {
	case RC_LOG_PROGRESS:
		ctx.m_messages[ctx.m_messageCount] = "PROG " + msg
	case RC_LOG_WARNING:
		ctx.m_messages[ctx.m_messageCount] = "WARN " + msg
	case RC_LOG_ERROR:
		ctx.m_messages[ctx.m_messageCount] = "ERR " + msg
	}
	ctx.m_messageCount++
}

func (ctx *BuildContext) doResetTimers() {
	for i := 0; i < RC_MAX_TIMERS; i++ {
		ctx.m_accTime[i] = time.Duration(0)
	}
}

func (ctx *BuildContext) doStartTimer(label rcTimerLabel) {
	ctx.m_startTime[label] = time.Now()
}

func (ctx *BuildContext) doStopTimer(label rcTimerLabel) {
	deltaTime := time.Now().Sub(ctx.m_startTime[label])
	if ctx.m_accTime[label] == 0 {
		ctx.m_accTime[label] = deltaTime
	} else {
		ctx.m_accTime[label] += deltaTime
	}
}

func (ctx *BuildContext) doGetAccumulatedTime(label rcTimerLabel) time.Duration {
	return ctx.m_accTime[label]
}
