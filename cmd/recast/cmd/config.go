// Copyright © 2017 NAME HERE <EMAIL ADDRESS>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package cmd

import (
	"fmt"

	"github.com/spf13/cobra"
)

// configCmd represents the config command
var configCmd = &cobra.Command{
	Use:   "config FILE",
	Short: "create a build settings file",
	Long: `Create a build settings file in YAML format, prefilled with default values.

If FILE is not provided, 'recast.yml' is used`,
	Run: func(cmd *cobra.Command, args []string) {
		// check user input
		path := "recast.yml"
		if len(args) >= 1 {
			path = args[0]
		}
		if ok, err := confirmIfExists(path,
			fmt.Sprintf("file name %s already exists, overwrite? [y/N]", path)); !ok {
			if err == nil {
				fmt.Println("aborted by user...")
			} else {
				fmt.Println("aborted,", err)
			}
			return
		}
		fmt.Printf("build settings written to '%s'\n", path)
	},
}

func init() {
	RootCmd.AddCommand(configCmd)

	// Here you will define your flags and configuration settings.

	// Cobra supports Persistent Flags which will work for this command
	// and all subcommands, e.g.:
	// configCmd.PersistentFlags().String("foo", "", "A help for foo")

	// Cobra supports local flags which will only run when this command
	// is called directly, e.g.:
	// configCmd.Flags().BoolP("toggle", "t", false, "Help message for toggle")
}
