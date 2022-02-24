#!/usr/bin/env python

import argparse
import os
import subprocess
import sys
 

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--objcopy',
                      help='Generate tool(eg: objcopy)',
                      metavar='FILE')  
  parser.add_argument('--output_format',
                      help='Create an output file in format',
                      metavar='FILE')             
  parser.add_argument('--input_file',
                      help='Input file',
                      metavar='FILE')
  args = parser.parse_args()
  
  (bin_name, bin_ext) = os.path.splitext(os.path.basename(args.input_file))
  command_string = '{tools} -O{oformat} {input_file} {output_file}'
  command = command_string.format(tools = args.objcopy,
                                  oformat = args.output_format,
                                  output_file = bin_name+'.bin',
                                  input_file = args.input_file)
  return subprocess.call(command, shell=True)
 
if __name__ == "__main__":
  sys.exit(main())
