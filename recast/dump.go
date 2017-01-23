package recast

import "time"

func logLine(ctx *BuildContext, label TimerLabel, name string, pc float64) {
	t := ctx.AccumulatedTime(label)
	if t < 0 {
		return
	}
	ctx.Progressf("%s:\t%.2fms\t(%.1f%%)", name, float64(t)/float64(time.Millisecond), float64(t)*pc)
}

func LogBuildTimes(ctx *BuildContext, totalTime time.Duration) {
	pc := 100.0 / float64(totalTime)
	ctx.Progressf("Build Times")
	logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES, "- Rasterize\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD, "- Build Compact\t\t", pc)
	logLine(ctx, RC_TIMER_FILTER_BORDER, "- Filter Border\t\t", pc)
	logLine(ctx, RC_TIMER_FILTER_WALKABLE, "- Filter Walkable\t\t", pc)
	logLine(ctx, RC_TIMER_ERODE_AREA, "- Erode Area\t\t", pc)
	logLine(ctx, RC_TIMER_MEDIAN_AREA, "- Median Area\t\t", pc)
	logLine(ctx, RC_TIMER_MARK_BOX_AREA, "- Mark Box Area\t\t", pc)
	logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA, "- Mark Convex Area\t\t", pc)
	logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA, "- Mark Cylinder Area\t", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD, "- Build Distance Field\t", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST, "    - Distance\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR, "    - Blur\t\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS, "- Build Regions\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED, "    - Watershed\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND, "      - Expand\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD, "      - Find Basins\t", pc)
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER, "    - Filter\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_LAYERS, "- Build Layers\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS, "- Build Contours\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE, "    - Trace\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY, "    - Simplify\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_POLYMESH, "- Build Polymesh\t\t", pc)
	logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL, "- Build Polymesh Detail\t", pc)
	logLine(ctx, RC_TIMER_MERGE_POLYMESH, "- Merge Polymeshes\t\t", pc)
	logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL, "- Merge Polymesh Details\t", pc)
	ctx.Progressf("=== TOTAL:\t%v", totalTime)
}
