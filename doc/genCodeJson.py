# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# genCodeJson.py
#
#  Created on: Sep 13, 2014
#      Author: Samir Menon <smenon@stanford.edu>
#
# The script runs through the src directory and reads in files and sub-directories
# into a json string that can be parsed and rendered in html webpages..
#
# This will export a set of json files to scl.git/doc/json/*.json

#!/bin/python
from os import walk
from os import path
from os import makedirs
import json

# Creates the json script for a list of files
def files2jsonstr(flist):
  ss = ""
  # Do nothing for an empty list
  if len(flist) == 0:
    return ss
  # Else add the json
  for f in flist:
    ss = ss+ "{ \"name\": \"" + f + "\",\"size\": " + str(1000) + " },"
  # Return the formatted string.
  return ss

# Creates the json script for a directory (called recursively)
def dir2jsonstr(dirbase, dirname, incl_files):
  f = []
  ss = ""
  # Get the files and subdirs
  for (dirpath, dirnames, filenames) in walk(dirbase+"/"+dirname):
    # Add this dir and set up children
    ss = "{\"name\": \"" + dirname + "\","
    if incl_files == 0:
      if len(dirnames) > 0:
        ss = ss + "\"children\":["
        # Add subdir json; has extra , @ end
        for dd in dirnames:
          ss = ss + dir2jsonstr(dirpath,dd,incl_files) + ","
        #Remove the extra , at the end of the string..
        ss = ss[:len(ss)-1] + " ]"; # Remove the , at the end..
      else:
        ss = ss + "\"size\": " + str(20000)
    else:
      if len(filenames) | len(dirnames) > 0:
        ss = ss + "\"children\":["
        
        # Format the files into a list
        f.extend(filenames)
        # Add the list of files as formatted json kids; has extra , @ end
        ss = ss + files2jsonstr(f)
        
        # Add subdir json; has extra , @ end
        for dd in dirnames:
          ss = ss + dir2jsonstr(dirpath,dd,incl_files) + ","
          
        #Remove the extra , at the end of the string..
        ss = ss[:len(ss)-1] + " ]"; # Remove the , at the end..
      else:
        ss = ss + "\"size\": " + str(20000)
    break
  ss = ss +" }"
  return ss

# Runs through the source folders and exports the json
mypath = "../src"
sutilpath = "../3rdparty/sUtil/src"

if not path.exists("./json"):
    makedirs("./json")

print("JSON for scl:")
ss = dir2jsonstr(mypath, 'scl', 1)
print(ss)
f = open('./json/scl.json','w')
f.write(ss)
f.close()

print("\n\n"+"JSON for scl_ext:")
ss = dir2jsonstr(mypath, 'scl_ext', 1)
print(ss)
f = open('./json/scl_ext.json','w')
f.write(ss)
f.close()

print("\n\n"+"JSON for scl and scl_ext and sutil:")
ss = "{\"name\": \"\",\"children\":["
ss = ss + dir2jsonstr(mypath, 'scl', 1) + ","
ss = ss + dir2jsonstr(mypath, 'scl_ext', 1) + ","
ss = ss + dir2jsonstr(sutilpath, 'sutil', 1)
ss = ss + "]}"
print(ss)
f = open('./json/src.json','w')
f.write(ss)
f.close()


print("\n\n"+"JSON for scl and scl_ext without files:")
ss = "{\"name\": \"\",\"children\":["
ss = ss + dir2jsonstr(mypath, 'scl', 0) + ","
ss = ss + dir2jsonstr(mypath, 'scl_ext', 0)
ss = ss + "]}"
print(ss)
f = open('./json/src_only_dirs.json','w')
f.write(ss)
f.close()

