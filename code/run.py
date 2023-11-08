#!/usr/bin/python2
from __future__ import print_function

DEBUG = True
SLEEP_TIME = 3 # in seconds

debug = lambda *args: print(*args) if DEBUG else lambda *args: None

import os
import fcntl
import sys
import time
import argparse
import stat
import signal

STDOUT = os.sys.stdout.fileno()
STDERR = os.sys.stderr.fileno()

DEFAULT_FILE_NAME = "commands.txt"
OUTPUT_DIR = "outputs"
NUM_OF_RUNS = 1
MUST_END_MARK = '#!'
PIPE_ENV_VAR = 'AIR2_PIPE_WRITE'
PIPE_R_ENV_VAR = 'AIR2_PIPE_READ'

def run_commands(v, theta, commands_file, compile=False, output_dir=OUTPUT_DIR, delete=False):
    # remove old shell files
    os.system("rm commands[0-9]*.sh > /dev/null 2> /dev/null")
    os.system("rm -rf outputs/* > /dev/null 2> /dev/null")

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
    print('chmod all files')
    for i, c in enumerate(commands_list):
        file_name = 'commands{i}.sh'.format(i=i)
        with open(file_name, 'w') as f:
            f.write(c.strip())
        os.chmod(file_name, os.stat(file_name).st_mode | stat.S_IEXEC)

    # run the commands, only if '-c' was not specified in the command line
    if not args.compile:
        # create pipe for reading the final positions:
        print('running')
        pipe_r, pipe_w = os.pipe()
        pipe_r2, pipe_w2 = os.pipe()
        fcntl.fcntl(pipe_r, fcntl.F_SETFL, os.O_NONBLOCK) 
        pipe_read_file = os.fdopen(pipe_r)
        # setting an environment variable for the pipe
        os.environ[PIPE_ENV_VAR] = "{}".format(pipe_w)
        os.environ[PIPE_R_ENV_VAR] = "{}".format(pipe_r2)
        os.write(pipe_w2, bytes("({},{})\n".format(v,theta))) # write v,theta to pipe in order to pass it to the simulation
        os.close(pipe_w2)
        must_end_pids = []
        shell_count = len(commands_list)
        pids = []
        os.makedirs(output_dir) if not os.path.exists(output_dir) else None
        for i in range(shell_count):
            print(i, end=' ')
            pid = os.fork()
            if pid > 0:
                pids.append(pid)
                if MUST_END_MARK in commands_list[i]:
                    must_end_pids.append(pid)
            else:
                os.close(pipe_r)
                # os.write(pipe_w, b'#')
                # os.set_inheritable(pipe_w, True)
                fdout = os.open(os.path.join(output_dir, 'output{i}.txt'.format(i=i)), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                fderr = os.open(os.path.join(output_dir, 'err{i}.txt'.format(i=i)), os.O_WRONLY|os.O_CREAT|os.O_TRUNC)
                os.dup2(fdout, STDOUT)
                os.dup2(fderr, STDERR)
                os.execv("/bin/bash", ["/bin/bash", 'commands{i}.sh'.format(i=i)]) # maybe add args?
            time.sleep(SLEEP_TIME)
        # wait the important chileds to die
        #TODO: check how to get data from the childs, maybe pips?
        print(must_end_pids)
        for pid in must_end_pids:
            _ = os.waitpid(pid, 0)
        print(pids)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
                print('killed', pid)
            except OSError:
                pass
        # read all the results
        os.close(pipe_w)
        res_str = pipe_read_file.read()
        print('data:',res_str)
        res = eval(res_str)
    if args.delete:
        # remove old shell files
        os.system("rm commands[0-9]*.sh > /dev/null 2> /dev/null")
    return res



# read args: commands file, whether to compile or run as well
args_parser = argparse.ArgumentParser()
args_parser.add_argument('commands_file', default=DEFAULT_FILE_NAME, nargs='?')
args_parser.add_argument('-c', '--compile', action='store_true', default=False)
args_parser.add_argument('-o', '--outputs', default=OUTPUT_DIR, nargs='?')
args_parser.add_argument('-n', '--number_of_runs', default=NUM_OF_RUNS, nargs='?')
args_parser.add_argument('-d', '--delete', action='store_true', default=False)
args = args_parser.parse_args()

# create shell files 

