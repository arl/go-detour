package cmd

import "github.com/spf13/cobra"

// buildCmd represents the build command
var buildCmd = &cobra.Command{
	Use:   "build OUTFILE",
	Short: "build navigation mesh from input geometry",
	Long: `Build a navigation mesh from input geometry in OBJ.
Build process is controlled by the provided build settings. Generated
navmesh is saved to OUTFILE in binary format, readable with go-detour
and/or detour.`,
	Run: func(cmd *cobra.Command, args []string) {
	},
}

var cfgVal, inputVal string

func init() {
	RootCmd.AddCommand(buildCmd)

	buildCmd.Flags().StringVar(&cfgVal, "config", "recast.yml", "build settings")
	buildCmd.Flags().StringVar(&typeVal, "type", "solo", "navmesh type, 'solo' or 'tiled'")
	buildCmd.Flags().StringVar(&inputVal, "input", "", "input geometry OBJ file (required)")
}
