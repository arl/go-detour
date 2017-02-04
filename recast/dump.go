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
	logLine(ctx, TimerRasterizeTriangles, "- Rasterize\t\t", pc)
	logLine(ctx, TimerBuildCompactHeightfield, "- Build Compact\t\t", pc)
	logLine(ctx, TimerFilterBorder, "- Filter Border\t\t", pc)
	logLine(ctx, TimerFilterWalkable, "- Filter Walkable\t\t", pc)
	logLine(ctx, TimerErodeArea, "- Erode Area\t\t", pc)
	logLine(ctx, TimerMedianArea, "- Median Area\t\t", pc)
	logLine(ctx, TimerMarkBoxArea, "- Mark Box Area\t\t", pc)
	logLine(ctx, TimerMarkConvexPolyArea, "- Mark Convex Area\t\t", pc)
	logLine(ctx, TimerMarkCylinderArea, "- Mark Cylinder Area\t", pc)
	logLine(ctx, TimerBuildDistanceField, "- Build Distance Field\t", pc)
	logLine(ctx, TimerBuildDistanceFieldDist, "    - Distance\t\t", pc)
	logLine(ctx, TimerBuildDistanceFieldBlur, "    - Blur\t\t\t", pc)
	logLine(ctx, TimerBuildRegions, "- Build Regions\t\t", pc)
	logLine(ctx, TimerBuildRegionsWatershed, "    - Watershed\t\t", pc)
	logLine(ctx, TimerBuildRegionsExpand, "      - Expand\t\t", pc)
	logLine(ctx, TimerBuildRegionsFlood, "      - Find Basins\t", pc)
	logLine(ctx, TimerBuildRegionsFilter, "    - Filter\t\t", pc)
	logLine(ctx, TimerBuildLayers, "- Build Layers\t\t", pc)
	logLine(ctx, TimerBuildContours, "- Build Contours\t\t", pc)
	logLine(ctx, TimerBuildContoursTrace, "    - Trace\t\t", pc)
	logLine(ctx, TimerBuildContoursSimplify, "    - Simplify\t\t", pc)
	logLine(ctx, TimerBuildPolymesh, "- Build Polymesh\t\t", pc)
	logLine(ctx, TimerBuildPolyMeshDetail, "- Build Polymesh Detail\t", pc)
	logLine(ctx, TimerMergePolymesh, "- Merge Polymeshes\t\t", pc)
	logLine(ctx, TimerMergePolyMeshDetail, "- Merge Polymesh Details\t", pc)
	ctx.Progressf("=== TOTAL:\t%v", totalTime)
}
