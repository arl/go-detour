package cmd

import (
	"fmt"
	"os"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/aurelien-rainone/go-detour/recast"
	"github.com/aurelien-rainone/go-detour/sample/solomesh"
	"github.com/aurelien-rainone/go-detour/sample/tilemesh"
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
	buildCmd.Flags().StringVar(&typeVal, "type", "solo", "navmesh type, 'solo' or 'tile'")
	buildCmd.Flags().StringVar(&inputVal, "input", "", "input geometry OBJ file (required)")
}

func doBuild(cmd *cobra.Command, args []string) {
	// check existence of input geometry flags
	if len(inputVal) == 0 {
		fmt.Printf("missing input geometry file (--input)")
		return
	}

	//
	// build navmesh
	//

	var (
		navMesh *detour.NavMesh
		err     error
		ok      bool
	)
	ctx := recast.NewBuildContext(true)

	switch typeVal {

	case "solo":

		// unmarshall build settings
		var cfg recast.BuildSettings
		err = unmarshalYAMLFile(cfgVal, &cfg)
		check(err)

		// read input geometry
		soloMesh := solomesh.New(ctx)
		var r *os.File
		r, err = os.Open(inputVal)
		check(err)
		defer r.Close()

		soloMesh.SetSettings(cfg)
		if err = soloMesh.LoadGeometry(r); err != nil {
			check(err)
		}
		navMesh, ok = soloMesh.Build()

	case "tile":

		// unmarshall build settings
		var cfg recast.BuildSettings
		err = unmarshalYAMLFile(cfgVal, &cfg)
		check(err)

		// read input geometry
		tileMesh := tilemesh.New(ctx)
		var r *os.File
		r, err = os.Open(inputVal)
		check(err)
		defer r.Close()

		tileMesh.SetSettings(cfg)
		if err = tileMesh.LoadGeometry(r); err != nil {
			check(err)
		}
		navMesh, ok = tileMesh.Build()

	default:
		fmt.Printf("unknown (or unimplemented) navmesh type '%v'\n", typeVal)
		return
	}

	ctx.DumpLog("")

	//
	// save
	//

	if !ok {
		fmt.Printf("couldn't build navmesh for %v\n", inputVal)
		return
	}

	// check output file name
	out := "navmesh.bin"
	if len(args) >= 1 {
		out = args[0]
	}
	if err = fileExists(out); err == nil {
		msg := fmt.Sprintf("\n'%v' already exists, overwrite? [y/N]", out)
		if overwrite := askForConfirmation(msg); !overwrite {
			fmt.Println("aborted")
			return
		}
	}

	err = navMesh.SaveToFile(out)
	check(err)

	fmt.Println("success")
	fmt.Printf("navmesh written to '%v'\n", out)
}
