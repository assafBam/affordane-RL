#!python3

import os

commands=open('commands.txt', 'r')
c=commands.read()
l=c.split('------------------------------------------------------')
for i, c in enumerate(l):
    with open(f'commands{i}.sh', 'w') as f:
        f.write(c.strip())
    os.system(f"chmod +r commands{i}.sh")
    
