#!/bin/python3

DEBUG = True
SLEEP_TIME = 1 # in seconds

debug = lambda *args: print(*args) if DEBUG else lambda *args: None

import os
import sys
import time

STDOUT = 1
STDERR = 2

DEFAULT_FILE_NAME = "commands.txt"

# remove old shell files
os.system("rm commands[0-9]*.sh")

# create shell files 

    # if a file name was provided, use it. otherwise, use the default
commands_file = sys.argv[1] if len(sys.argv)>1 else DEFAULT_FILE_NAME
commands=open(commands_file, 'r')
tmp=""
commands_list=[]
for line in commands:
    if line.startswith('---'):
        commands_list.append(tmp)
        tmp=""
        continue
    tmp+="\n"+line
commands_list.append(tmp)
for i, c in enumerate(commands_list):
    with open(f'commands{i}.sh', 'w') as f:
        f.write(c.strip())
    os.system(f"chmod +r commands{i}.sh")

# run the commands
shell_count: int = len(commands_list)
pids = []
for i in range(shell_count):
    pid = os.fork()
    if pid > 0:
        pids.append(pid)
    else:
        fdout = os.open(f'output{i}.txt', os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
        fderr = os.open(f'err{i}.txt', os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
        os.dup2(fdout, STDOUT)
        os.dup2(fderr, STDERR)
        os.execv("/bin/bash", ["/bin/bash", f"commands{i}.sh"]) # maybe add args?
    time.sleep(SLEEP_TIME)
# wait for the last one(?) to end
# run again?
# add config file
    # wait for "two"? children to exit and kill al the other children