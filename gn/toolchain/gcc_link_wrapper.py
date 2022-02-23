#!/usr/bin/env python
# Copyright 2015 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

"""Runs a linking command and optionally a strip command.

This script exists to avoid using complex shell commands in
gcc_toolchain.gni's tool("link"), in case the host running the compiler
does not have a POSIX-like shell (e.g. Windows).
"""

import argparse
import os
import subprocess
import sys

#import wrapper_utils


# When running on a Windows host and using a toolchain whose tools are
# actually wrapper scripts (i.e. .bat files on Windows) rather than binary
# executables, the "command" to run has to be prefixed with this magic.
# The GN toolchain definitions take care of that for when GN/Ninja is
# running the tool directly.  When that command is passed in to this
# script, it appears as a unitary string but needs to be split up so that
# just 'cmd' is the actual command given to Python's subprocess module.
BAT_PREFIX = 'cmd /c call '

def CommandToRun(command):
  if command[0].startswith(BAT_PREFIX):
    command = command[0].split(None, 3) + command[1:]
  return command


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--size',
                      help='Generate build information',
                      metavar='FILE')  
  parser.add_argument('--objcopy',
                      help='Generate binary file',
                      metavar='FILE')             
  parser.add_argument('--syms',
                      help='Generate symbols',
                      metavar='FILE') 
  parser.add_argument('--symout',
                      help='Generate symbols options',
                      metavar='FILE')    
  parser.add_argument('--ccflags',
                      help='Compile flags',
                      metavar='FILE')        
  parser.add_argument('--output',
                      required=False,
                      help='Final output executable file',
                      metavar='FILE')
  parser.add_argument('command', nargs='+',
                      help='Linking command')

  args = parser.parse_args()

  # Work-around for gold being slow-by-default. http://crbug.com/632230
  fast_env = dict(os.environ)
  result = subprocess.call(args.command, env=fast_env)
  if result != 0:
    return result

  if args.syms:
    compile_cmd = args.syms + "\"" + args.ccflags + "\"" + args.symout
    result = subprocess.call(compile_cmd, shell=True)
    if result != 0:
      return result
    args.command.append('ksym.o')
    result = subprocess.call(args.command, env=fast_env)
    if result != 0:
      return result
   
  if args.size:
    print(CommandToRun([args.size]))
    result = subprocess.call([args.size, args.output], env=fast_env)
    if result != 0:
      return result
 
  if args.objcopy:
    result = subprocess.call(args.objcopy.split(None, -1))
    if result != 0:
      return result

  return result



if __name__ == "__main__":
  sys.exit(main())
