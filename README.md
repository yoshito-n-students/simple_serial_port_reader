# simple_serial_port_reader
Read, search, and format texts from a serial port

## <u>Published topics</u>
___formatted___ (std_msgs/String)
* formatted texts

## <u>Parameters</u>
___~device___ (string, default: "/dev/ttyUSB0")

___~baudrate___ (int, default: 9600)

___~start_command___ (string, default: "")
* command to write just after opening the serial port
* write nothing if empty
* do not miss adding newline charactors if required

___~match_expression___ (string, default: "(.+)\r?\n")
* regular expression to match read texts
* the default matches each entire line

___~format_expression___ (string, default: "$1")
* format string to be applied to match results
* the default extracts 1st matched group in the match expression (i.e. with the default match expression, all charactors except newline charactors will be extracted)

___~stop_command___ (string, default: "")
* command to write just before closing the serial port
* write nothing if empty
* do not miss adding newline charactors if required

___~verbose___ (bool, default: false)
* print texts wrote, read and formatted

## <u>Example</u>
* See [example.launch](launch/example.launch)