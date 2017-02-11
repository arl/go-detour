package cmd

import (
	"encoding/json"
	"fmt"
	"os"

	"github.com/aurelien-rainone/go-detour/detour"
	"github.com/spf13/cobra"
)

// infosCmd represents the infos command
var infosCmd = &cobra.Command{
	Use:   "infos NAVMESH",
	Short: "show infos about a navmesh",
	Long: `Read a navigation mesh from binary file, check the data
for consistency then print informations on standard output.`,
	Run: doInfos,
}

func init() {
	RootCmd.AddCommand(infosCmd)
}

func doInfos(cmd *cobra.Command, args []string) {
	// check existence of navmesh
	if len(args) < 1 {
		fmt.Printf("no input navmesh file")
		return
	}
	var (
		err     error
		binMesh string
		navmesh *detour.NavMesh
	)
	binMesh = args[0]

	// read navmesh
	var f *os.File
	f, err = os.Open(binMesh)
	check(err)
	defer f.Close()

	// decode navmesh
	navmesh, err = detour.Decode(f)
	check(err)

	// marshall navmesh params to json
	var buf []byte
	buf, err = json.MarshalIndent(navmesh.Params, "", "  ")
	check(err)
	fmt.Printf("successfully loaded '%v'\n", binMesh)
	fmt.Printf("'%v' navmesh infos:\n%s", typeVal, string(buf))
}
