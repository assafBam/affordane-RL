#!/usr/bin/python3
from __future__ import print_function

DEBUG = True
SLEEP_TIME = 0.1 # in seconds

debug = lambda *args: print(*args) if DEBUG else lambda *args: None

import os
import sys
import time
import argparse
import stat
import signal
from tqdm import tqdm

STDOUT = os.sys.stdout.fileno()
STDERR = os.sys.stderr.fileno()

DEFAULT_FILE_NAME = "commands.txt"
OUTPUT_DIR = "outputs"
NUM_OF_RUNS = 1
MUST_END_MARK = '#!'
PIPE_ENV_VAR = 'AIR2_PIPE_WRITE'

# remove old shell files
os.system("rm commands[0-9]*.sh > /dev/null 2> /dev/null")
os.system("rm -rf outputs/* > /dev/null 2> /dev/null")

# read args: commands file, whether to compile or run as well
args_parser = argparse.ArgumentParser()
args_parser.add_argument('commands_file', default=DEFAULT_FILE_NAME, nargs='?')
args_parser.add_argument('-c', '--compile', action='store_true', default=False)
args_parser.add_argument('-o', '--outputs', default=OUTPUT_DIR, nargs='?')
args_parser.add_argument('-n', '--number_of_runs', default=NUM_OF_RUNS, nargs='?')
args_parser.add_argument('-d', '--delete', action='store_true', default=False)
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
    file_name = 'commands{i}.sh'.format(i=i)
    with open(file_name, 'w') as f:
        f.write(c.strip())
    os.chmod(file_name, os.stat(file_name).st_mode | stat.S_IEXEC)

# run the commands, only if '-c' was not specified in the command line
if not args.compile:
    # create pipe for reading the final positions:
    pipe_r, pipe_w = os.pipe2(os.O_NONBLOCK)
    # setting an environment variable for the pipe
    os.environ[PIPE_ENV_VAR] = "{}".format(pipe_w)

    for j in tqdm(range(int(args.number_of_runs))):
        if int(args.number_of_runs) <= 1:
            out_dir = args.outputs
        else:
            out_dir = os.path.join(args.outputs, str(j))
        must_end_pids = []
        shell_count = len(commands_list)
        pids = []
        os.makedirs(out_dir) if not os.path.exists(out_dir) else None
        for i in range(shell_count):
            pid = os.fork()
            if pid > 0:
                pids.append(pid)
                if MUST_END_MARK in commands_list[i]:
                    must_end_pids.append(pid)
            else:
                os.close(pipe_r)
                # os.set_inheritable(pipe_w, True)
                fdout = os.open(os.path.join(out_dir, 'output{i}.txt'.format(i=i)), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                fderr = os.open(os.path.join(out_dir, 'err{i}.txt'.format(i=i)), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                os.dup2(fdout, STDOUT)
                os.dup2(fderr, STDERR)
                os.execv("/bin/bash", ["/bin/bash", 'commands{i}.sh'.format(i=i)]) # maybe add args?
            time.sleep(SLEEP_TIME)
        # wait the important chileds to die
        #TODO: check how to get data from the childs, maybe pips?
        for pid in must_end_pids:
            _ = os.waitpid(pid, 0)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    # read all the results
    os.close(pipe_w)
    pipe_read_file = os.fdopen(pipe_r)
    print(pipe_read_file.read()) #for now, print all the information. TODO: do something with the results

if args.delete:
    # remove old shell files
    os.system("rm commands[0-9]*.sh > /dev/null 2> /dev/null")