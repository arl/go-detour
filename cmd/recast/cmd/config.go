package cmd

import (
	"fmt"
	"os"

	yaml "gopkg.in/yaml.v2"

	"github.com/aurelien-rainone/go-detour/sample/solomesh"
	"github.com/spf13/cobra"
)

// configCmd represents the config command
var configCmd = &cobra.Command{
	Use:   "config --type TYPE FILE",
	Short: "generate a config file with default build settings",
	Long: `Generate a build config file in YAML format, pre-filled with the
default settings to build a navmesh of type TYPE.

If FILE is not provided, "recast.yml" is used.
If TYPE is not provided, it defaults to "solo", single-tile navigation mesh.

To use the generated file, simply call "recast build --cfg FILE".`,
	Run: func(cmd *cobra.Command, args []string) {

		// check navmesh type
		var cfg []byte
		var ok bool
		if cfg, ok = defaultCfgs[typeVal]; !ok {
			fmt.Printf("aborted, can't generate default config for '%s' navmesh\n", typeVal)
			return
		}

		// check file name
		path := "recast.yml"
		if len(args) >= 1 {
			path = args[0]
		}
		if ok, err := confirmIfExists(path,
			fmt.Sprintf("file name %s already exists, overwrite? [y/N]", path)); !ok {
			if err == nil {
				fmt.Println("aborted...")
			} else {
				fmt.Println("aborted,", err)
			}
			return
		}

		// write config
		f, err := os.Create(path)
		if err != nil {
			fmt.Println("error:", err)
			return
		}
		defer f.Close()

		_, err = f.Write(cfg)
		if err != nil {
			fmt.Println("error:", err)
			return
		}

		fmt.Printf("build settings for %s navmesh generated to '%s'\n", typeVal, path)
	},
}

var (
	defaultCfgs = make(map[string][]byte)
	typeVal     string
)

func init() {
	RootCmd.AddCommand(configCmd)

	configCmd.Flags().StringVar(&typeVal, "type", "solo", "Type of navigation mesh (solo, tiled)")

	// register solo mesh configs
	if buf, err := yaml.Marshal(solomesh.NewSettings()); err != nil {
		fmt.Println("couldn't register solomesh default settings,", err)
	} else {
		defaultCfgs["solo"] = buf
	}

	// navmesh default type flag
	typeVal = "solo"
}
