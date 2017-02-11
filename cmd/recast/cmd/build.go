package cmd

import (
	"fmt"
	"io/ioutil"

	yaml "gopkg.in/yaml.v2"

	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/go-detour/sample/solomesh"
	"github.com/spf13/cobra"
)

// buildCmd represents the build command
var buildCmd = &cobra.Command{
	Use:   "build OUTFILE",
	Short: "build navigation mesh from input geometry",
	Long: `Build a navigation mesh from input geometry in OBJ.
Build process is controlled by the provided build settings. Generated
navmesh is saved to OUTFILE in binary format, readable with go-detour
and/or detour.`,
	Run: doBuild,
}

var cfgVal, inputVal string

func init() {
	RootCmd.AddCommand(buildCmd)
	buildCmd.Flags().StringVar(&cfgVal, "config", "recast.yml", "build settings")
	buildCmd.Flags().StringVar(&typeVal, "type", "solo", "navmesh type, 'solo' or 'tiled'")
	buildCmd.Flags().StringVar(&inputVal, "input", "", "input geometry OBJ file (required)")
}

func doBuild(cmd *cobra.Command, args []string) {
	// check existence of input geometry
	if len(inputVal) == 0 {
		fmt.Printf("missing input geometry file (--input)")
		return
	}
	var err error
	err = fileExists(inputVal)
	check(err)

	// check output file name
	out := "navmesh.bin"
	if len(args) >= 1 {
		out = args[0]
	}
	if err = fileExists(out); err == nil {
		msg := fmt.Sprintf("'%v' already exists, overwrite? [y/N]", out)
		if overwrite := askForConfirmation(msg); !overwrite {
			fmt.Println("aborted")
			return
		}
	}

	switch typeVal {

	case "solo":
		var (
			buf []byte
			cfg solomesh.Settings
		)
		// read config
		buf, err = ioutil.ReadFile(cfgVal)
		check(err)

		// marhsall config
		err = yaml.Unmarshal(buf, &cfg)
		check(err)

		// read input geometry
		ctx := recast.NewBuildContext(true)
		soloMesh := solomesh.New(ctx)
		if err = soloMesh.LoadGeometry(inputVal); err != nil {
			ctx.DumpLog("")
			check(err)
		}
		navMesh, ok := soloMesh.Build()
		if !ok {
			ctx.DumpLog("")
			fmt.Printf("couldn't build navmesh for %v\n", inputVal)
			return
		}

		err = navMesh.SaveToFile(out)
		check(err)

		fmt.Println("success")
		ctx.DumpLog("'%v' navmesh generated '%v'\n", typeVal, out)
	default:
		fmt.Printf("unknown (or unimplemented) navmesh type '%v'\n", typeVal)
	}
}
