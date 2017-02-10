package cmd

import "github.com/spf13/cobra"

// infosCmd represents the infos command
var infosCmd = &cobra.Command{
	Use:   "infos NAVMESH",
	Short: "show infos about a navmesh",
	Long: `Read a navigation mesh from binary file, check the data
for consistency then print informations on standard output.`,
	Run: doInfos,
}

func init() {
	RootCmd.AddCommand(infoCmd)
	buildCmd.Flags().StringVar(&typeVal, "type", "solo", "navmesh type, 'solo' or 'tiled'")
}

func doInfos(cmd *cobra.Command, args []string) {
}
