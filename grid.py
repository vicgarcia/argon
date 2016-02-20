import sys
import argon.argsparse as argsparse
from argon.grid import do_grid


# combine arguments as string
args = ' '.join(sys.argv)

# output the grid points
do_grid(args)
