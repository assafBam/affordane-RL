from __future__ import print_function
import os

###########################################################
########### log the position of the ball helper ###########
PIPE_ENV_VAR = 'AIR2_PIPE_WRITE'
import os
pipe_write_fd = int(os.environ.get(PIPE_ENV_VAR, os.sys.stdout.fileno()))

def final_position_logger(x):
    os.write(pipe_write_fd, bytes(str(x)+'\n', 'ascii'))

###########################################################

final_position_logger(15.333)