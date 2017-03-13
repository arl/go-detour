package cmd

import (
	"bufio"
	"fmt"
	"io/ioutil"
	"os"

	yaml "gopkg.in/yaml.v2"
)

// convenience function that returns nil if file exists, or an error if it
// doesn't or if file can't be stat'ed
func fileExists(path string) (err error) {
	if _, err = os.Stat(path); err != nil {
		if os.IsNotExist(err) {
			// file does not exist
			err = fmt.Errorf("no such file '%v'", path)
		}
	}
	return err
}

// askForConfirmation show msg and ask for the user to type y or n
// (typing ENTER defaults to no)
func askForConfirmation(msg string) bool {
	fmt.Println(msg)
	reader := bufio.NewReader(os.Stdin)
	defaultInput := byte('N')

	for {
		input, _ := reader.ReadString('\n')
		c := string([]byte(input)[0])[0]
		if c == 10 {
			// ENTER
			c = defaultInput
		}
		switch c {
		case 'Y', 'y':
			return true
		case 'N', 'n':
			return false
		}
	}
}

func check(err error) {
	if err != nil {
		fmt.Printf("error, %v\n", err)
		os.Exit(-1)
	}
}

func unmarshalYAMLFile(path string, out interface{}) error {
	buf, err := ioutil.ReadFile(path)
	if err != nil {
		return err
	}
	return yaml.Unmarshal(buf, out)
}
