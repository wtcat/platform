#!/usr/bin/env python
# Copyright (c) 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

from __future__ import print_function

import json
import os
import subprocess
import sys
import re
from optparse import OptionParser

# This script runs pkg-config, optionally filtering out some results, and
# returns the result.
#
# The result will be [ <includes>, <cflags>, <libs>, <lib_dirs>, <ldflags> ]
# where each member is itself a list of strings.
#
# You can filter out matches using "-v <regexp>" where all results from
# pkgconfig matching the given regular expression will be ignored. You can
# specify more than one regular expression my specifying "-v" more than once.
#
# You can specify a sysroot using "-s <sysroot>" where sysroot is the absolute
# system path to the sysroot used for compiling. This script will attempt to
# generate correct paths for the sysroot.
#
# When using a sysroot, you must also specify the architecture via
# "-a <arch>" where arch is either "x86" or "x64".
#
# CrOS systemroots place pkgconfig files at <systemroot>/usr/share/pkgconfig
# and one of <systemroot>/usr/lib/pkgconfig or <systemroot>/usr/lib64/pkgconfig
# depending on whether the systemroot is for a 32 or 64 bit architecture. They
# specify the 'lib' or 'lib64' of the pkgconfig path by defining the
# 'system_libdir' variable in the args.gn file. pkg_config.gni communicates this
# variable to this script with the "--system_libdir <system_libdir>" flag. If no
# flag is provided, then pkgconfig files are assumed to come from
# <systemroot>/usr/lib/pkgconfig.
#
# Additionally, you can specify the option --atleast-version. This will skip
# the normal outputting of a dictionary and instead print true or false,
# depending on the return value of pkg-config for the given package.


rtems_compile_options = [
  '--cflags',
  '--libs-only-other',
  '--variable=prefix',
  '--variable=RTEMS_ARCH',
  '--variable=RTEMS_BSP',
  '--variable=RTEMS_MAJOR'
]

def find_package_name(path):
  flist = []
  for root,dirs,files in os.walk(path):
    for file in files:
      fname, fext = os.path.splitext(file)
      if fext == '.pc':
        return fname
  return ''

def main():
  # If this is run on non-Linux platforms, just return nothing and indicate
  # success. This allows us to "kind of emulate" a Linux build from other
  # platforms.
  if "linux" not in sys.platform:
    print("[ , , , , ]")
    return 0

  parser = OptionParser()
  parser.add_option('-d', '--debug', action='store_true')
  parser.add_option('-p', action='store', dest='pkg_config', type='string',
                    default='pkg-config')
  parser.add_option('-v', action='append', dest='strip_out', type='string')
  parser.add_option('--path', action='store', dest='path', type='string')
  (options, args) = parser.parse_args()

  base_cmd = []
  if options.path:
    pkgname = find_package_name(options.path)
    if not pkgname:
      print('Error***: Not found', options.path)
      return -1
    os.environ['PKG_CONFIG_PATH'] = options.path
    base_cmd.append(options.pkg_config)
    base_cmd.append(pkgname)
    base_cmd += args

  # Make a list of regular expressions to strip out.
  flag_string = []
  for key in rtems_compile_options:
    try:
      cmd = base_cmd + [key]
      if options.debug:
        sys.stderr.write('Running: %s\n' % ' '.join(cmd))
      flag_string.append(subprocess.check_output(cmd).decode('utf-8'))
    except:
      sys.stderr.write('Could not run pkg-config.\n')
      return 1
  # For now just split on spaces to get the args out. This will break if
  # pkgconfig returns quoted things with spaces in them, but that doesn't seem
  # to happen in practice.
 # all_flags = flag_string.strip().split(' ')

  # Output a GN array, the first one is the cflags, the second are the libs. The
  # JSON formatter prints GN compatible lists when everything is a list of
  # strings.
  print(json.dumps([flag_string[0].strip().split(' '), 
    flag_string[1].strip().split('\n')[0], 
    flag_string[2].strip().split('\n')[0], 
    flag_string[3].strip().split('\n')[0], 
    flag_string[4].strip().split('\n')[0], 
    flag_string[5].strip().split('\n')[0]])
    )
  return 0


if __name__ == '__main__':
  sys.exit(main())
