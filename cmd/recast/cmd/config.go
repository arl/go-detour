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
	Use:   "config [FILE]",
	Short: "generate a config file with default build settings",
	Long: `Write to FILE a build config in YAML format, pre-filled with the
default settings to build a navmesh of type TYPE.

To use the generated file, call "recast build --cfg FILE".`,
	Run: doConfig,
}

var (
	defaultCfgs = make(map[string][]byte)
	typeVal     string
)

func init() {
	RootCmd.AddCommand(configCmd)
	configCmd.Flags().StringVar(&typeVal, "type", "solo", "navmesh type, 'solo' or 'tiled'")

	// register solo mesh configs
	if buf, err := yaml.Marshal(solomesh.NewSettings()); err != nil {
		fmt.Println("couldn't register solomesh default settings,", err)
	} else {
		defaultCfgs["solo"] = buf
	}

	// navmesh default type flag
	typeVal = "solo"
}

func doConfig(cmd *cobra.Command, args []string) {
	// check navmesh type
	var (
		cfg []byte
		ok  bool
		err error
	)

	if cfg, ok = defaultCfgs[typeVal]; !ok {
		fmt.Printf("error, unknown (or unimplemented) navmesh type '%v'\n", typeVal)
		return
	}

	// check file name
	path := "recast.yml"
	if len(args) >= 1 {
		path = args[0]
	}
	if err = fileExists(path); err == nil {
		msg := fmt.Sprintf("\n'%v' already exists, overwrite? [y/N]", path)
		if overwrite := askForConfirmation(msg); !overwrite {
			fmt.Println("aborted")
			return
		}
	}

	// write config
	f, err := os.Create(path)
	check(err)
	defer f.Close()
	_, err = f.Write(cfg)
	check(err)

	fmt.Printf("success\n")
	fmt.Printf("build settings for '%v' navmesh generated to '%v'\n", typeVal, path)
}
