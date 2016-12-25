package recast

import "time"

func logLine(ctx *Context, label TimerLabel, name string, pc float64) {
	t := ctx.AccumulatedTime(label)
	if t < 0 {
		return
	}
	ctx.Progressf("%s:\t%.2fms\t(%.1f%%)", name, t*time.Millisecond, t*time.Duration(pc))
}

func LogBuildTimes(ctx *Context, totalTimeUsec time.Duration) {
	pc := 100.0 / float64(totalTimeUsec)

	ctx.Progressf("Build Times")
	logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES, "- Rasterize", pc)
	logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD, "- Build Compact", pc)
	logLine(ctx, RC_TIMER_FILTER_BORDER, "- Filter Border", pc)
	logLine(ctx, RC_TIMER_FILTER_WALKABLE, "- Filter Walkable", pc)
	logLine(ctx, RC_TIMER_ERODE_AREA, "- Erode Area", pc)
	logLine(ctx, RC_TIMER_MEDIAN_AREA, "- Median Area", pc)
	logLine(ctx, RC_TIMER_MARK_BOX_AREA, "- Mark Box Area", pc)
	logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA, "- Mark Convex Area", pc)
	logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA, "- Mark Cylinder Area", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD, "- Build Distance Field", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST, "    - Distance", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR, "    - Blur", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS, "- Build Regions", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED, "    - Watershed", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND, "      - Expand", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD, "      - Find Basins", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER, "    - Filter", pc)
	logLine(ctx, RC_TIMER_BUILD_LAYERS, "- Build Layers", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS, "- Build Contours", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE, "    - Trace", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY, "    - Simplify", pc)
	logLine(ctx, RC_TIMER_BUILD_POLYMESH, "- Build Polymesh", pc)
	logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL, "- Build Polymesh Detail", pc)
	logLine(ctx, RC_TIMER_MERGE_POLYMESH, "- Merge Polymeshes", pc)
	logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL, "- Merge Polymesh Details", pc)
	ctx.Progressf("=== TOTAL:\t%.2fms", totalTimeUsec/1000.0)
}
