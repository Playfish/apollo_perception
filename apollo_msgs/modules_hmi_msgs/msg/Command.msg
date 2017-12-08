string name

#Command components, in [cmd, arg1, arg2, ...] format.
#Binary path starting with '/' indicates an absolute path, otherwise it will
#be taken as a relative path of the Apollo root.
string[] command

#Log file paths.
string stdout_file #[default = "/dev/null"]
string stderr_file #[default = "/dev/null"]
