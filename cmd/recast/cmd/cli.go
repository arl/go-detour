package cmd

import (
	"bufio"
	"fmt"
	"os"
)

// confirmIfExists checks that a file exists, and ask the user confirmation to
// do go forward.
//
// It returns true if the file doesn't exist, or if the user answered yes to the
// confirmation msg showed on command line. If ok is false or err is not nil,
// the operation on path should be aborted.
func confirmIfExists(path, msg string) (ok bool, err error) {
	if _, err := os.Stat(path); err != nil {
		if os.IsNotExist(err) {
			// file does not exist
			return true, nil
		} else {
			// other error
			fmt.Println("other error", err)
			return false, err
		}
	}
	return askForConfirmation(msg), nil
}

// askForConfirmation show msg and ask for the user to type y or n (typing ENTER
// default to no)
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
