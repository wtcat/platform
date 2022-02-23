#!/usr/bin/env python

import argparse
import os
import subprocess
import sys
 

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--arch',
                      help='set architecture to arch',
                      metavar='FILE')  
  parser.add_argument('--os',
                      help='set operating system to os',
                      metavar='FILE')             
  parser.add_argument('--image',
                      help='set image type to type',
                      metavar='FILE') 
  parser.add_argument('--compression',
                      help='set compression type comp',
                      metavar='FILE')    
  parser.add_argument('--loadaddr',
                      help='set load address to addr (hex)',
                      metavar='FILE')        
  parser.add_argument('--entryaddr',
                      help='set entry point to ep (hex)',
                      metavar='FILE')
  parser.add_argument('--name',
                      help='set image name to name',
                      metavar='FILE')
  parser.add_argument('--binfile',
                      help='use image data from datafile',
                      metavar='FILE')
  args = parser.parse_args()
  
  (bin_name, bin_ext) = os.path.splitext(os.path.basename(args.binfile))
  command_string = 'mkimage -A {arch} -O {os} -T {image} -C {comp} -a {load} -e {entry} -n {name} -d {input_file} {output_file}'
  command = command_string.format(arch=args.arch, 
                                  os = args.os, 
                                  image = args.image, 
                                  comp = args.compression,
                                  load = hex(int(args.loadaddr)),
                                  entry = hex(int(args.entryaddr)),
                                  name = args.name,
                                  input_file = args.binfile,
                                  output_file = bin_name + '.img')
  return subprocess.call(command, shell=True)
 
if __name__ == "__main__":
  sys.exit(main())
