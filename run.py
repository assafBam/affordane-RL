#!/usr/bin/python3

DEBUG = True
SLEEP_TIME = 1 # in seconds

debug = lambda *args: print(*args) if DEBUG else lambda *args: None

import os
import sys
import time
import argparse
import stat
import signal

STDOUT = 1
STDERR = 2

DEFAULT_FILE_NAME = "commands.txt"
OUTPUT_DIR = "outputs"
NUM_OF_RUNS = 1
MUST_END_MARK = '#!'

# remove old shell files
os.system("rm commands[0-9]*.sh > /dev/null")

# read args: commands file, whether to compile or run as well
args_parser = argparse.ArgumentParser()
args_parser.add_argument('commands_file', default=DEFAULT_FILE_NAME, nargs='?')
args_parser.add_argument('-c', '--compile', action='store_true', default=False)
args_parser.add_argument('-o', '--outputs', default=OUTPUT_DIR, nargs='?')
args_parser.add_argument('-n', '--number_of_runs', default=NUM_OF_RUNS, nargs='?')
args = args_parser.parse_args()

# create shell files 

commands=open(args.commands_file, 'r')
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
    file_name = f'commands{i}.sh'
    with open(file_name, 'w') as f:
        f.write(c.strip())
    os.chmod(file_name, os.stat(file_name).st_mode | stat.S_IEXEC)

# run the commands, only if '-c' was not specified in the command line
if not args.compile:
    for _ in range(args.number_of_runs):
        must_end_pids = []
        shell_count: int = len(commands_list)
        pids = []
        os.makedirs(args.outputs) if not os.path.exists(args.outputs) else None
        for i in range(shell_count):
            pid = os.fork()
            if pid > 0:
                pids.append(pid)
                if MUST_END_MARK in commands_list[i]:
                    must_end_pids.append(pid)
            else:
                fdout = os.open(os.path.join(args.outputs, f'output{i}.txt'), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                fderr = os.open(os.path.join(args.outputs, f'err{i}.txt'), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                os.dup2(fdout, STDOUT)
                os.dup2(fderr, STDERR)
                os.execv("/bin/bash", ["/bin/bash", f"commands{i}.sh"]) # maybe add args?
            time.sleep(SLEEP_TIME)
        # wait the important chileds to die
        for pid in must_end_pids:
            _ = os.waitpid(pid, 0)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
