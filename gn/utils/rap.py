#!/usr/bin/env python

import argparse
import os
import subprocess
import sys
 
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

  command_string = 'rtems-ld --base {base} {input} --format {format}  --output {output} {libpath} {libs}'
  command = command_string.format(base   = args.base, 
                                  input  = args.input,
                                  format = args.format, 
                                  output = args.output, 
                                  libpath = args.libpath,
                                  libs = args.libs)
  print("########", command)
  return subprocess.call(command, shell=True)
 
if __name__ == "__main__":
  sys.exit(main())
