#!/usr/bin/env python

import argparse
import os
import subprocess
import sys

def find_files(path):
    result_list = []
    for root, ds, fs in os.walk(path):
        for f in fs:
            result_list.append(os.path.join(root, f))
    return result_list

def files_filter(path, *k):
    flist = find_files(path)
    filtered = []
    for f in flist:
        fext = os.path.splitext(f)
        for ext in k:
            if ext == fext[1]:
                filtered.append(f)
                break
    return filtered

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--base',
                      help='Base kernel image',
                      metavar='FILE') 
  parser.add_argument('--input',
                      help='Input files',
                      metavar='FILE')  
  parser.add_argument('--format',
                      help='Output format for file',
                      metavar='FILE')             
  parser.add_argument('--output',
                      help='Output target name',
                      metavar='FILE') 
  parser.add_argument('--libpath',
                      help='Libraries path',
                      metavar='FILE')    
  parser.add_argument('--libs',
                      help='Libraries',
                      metavar='FILE')        
  args = parser.parse_args()

  # Find input object files
  input_files = files_filter(args.input, '.o')
  files_str = ' '
  for file in input_files:
    files_str = files_str + file + ' '

  # Convert libs
  if args.libs != '*':
    dep_libs = ' -l' + args.libs
  else:
    dep_libs = ' '

  # Convert libpath
  if args.libpath != '*':
    dep_path = ' -L' + args.libpath
  else:
    dep_path = ' '

  # Construct command 
  command_string = 'rtems-ld --base {base} {input} --out-format {format}  --output {output} {libs} {libpath}'
  command = command_string.format(base   = args.base, 
                                  input  = files_str,
                                  format = args.format, 
                                  output = args.output,
                                  libs   = dep_libs,
                                  libpath = dep_path)
  # print('LinkCommand: ', command)
  return subprocess.call(command, shell=True)
 
if __name__ == "__main__":
  sys.exit(main())
