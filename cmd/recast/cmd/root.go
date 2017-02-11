package cmd

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"
)

var cfgFile string

// RootCmd represents the base command when called without any subcommands
var RootCmd = &cobra.Command{
	Use:   "recast",
	Short: "build recast navmeshes",
	Long: `This is the command-line application accompanying go-detour:
	- build navigation meshes from any level geometry,
	- save them to binary files (usable in 'go-detour')
	- easily tweak build settings (YAML files),
	- check or show info about generated navmesh binaries.`,
}

// Execute adds all child commands to the root command sets flags appropriately.
// This is called by main.main(). It only needs to happen once to the rootCmd.
func Execute() {
	if err := RootCmd.Execute(); err != nil {
		fmt.Println(err)
		os.Exit(-1)
	}
}
