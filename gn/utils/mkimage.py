#!/usr/bin/env python

import argparse
import os
import subprocess
import sys
 

compress_table = {
  'gzip':  '.bin.gz',
  'bzip2': '.bin.bz2',
  'lz4':   '.bin.lz4',
  'lzma':  '.bin.lzma',
  'lzo':   '.bin.lzo',
  'zstd':  '.bin.std',
  'none':  '.bin'  
}

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
  input_binfile = bin_name + compress_table[args.compression]
  command_string = 'mkimage -A {arch} -O {os} -T {image} -C {comp} -a {load} -e {entry} -n {name} -d {input_file} {output_file}'
  command = command_string.format(arch=args.arch, 
                                  os = args.os, 
                                  image = args.image, 
                                  comp = args.compression,
                                  load = hex(int(args.loadaddr)),
                                  entry = hex(int(args.entryaddr)),
                                  name = args.name,
                                  input_file = input_binfile,
                                  output_file = bin_name + '.img')
  if args.compression == "gzip":
    gzip_string = 'gzip -c {srcfile} > {dstfile}'.format(srcfile = args.binfile, dstfile = input_binfile)
    ret = subprocess.call(gzip_string, shell=True)
    if ret:
      return ret
  elif args.compression == "none":
    pass

  return subprocess.call(command, shell=True)
 
if __name__ == "__main__":
  sys.exit(main())
