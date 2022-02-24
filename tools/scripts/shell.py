#!/usr/bin/env python

import argparse
import subprocess
import sys

def main():
  print(sys.argv[0])
  parser = argparse.ArgumentParser()       
  parser.add_argument('command', nargs='+',
                      help='Executable command')
  args = parser.parse_args()
  for cmd in args.command:
    ret = subprocess.call(cmd, shell=True)
    if ret != 0:
      return ret;
  return 0
 
if __name__ == "__main__":
  sys.exit(main())
