#!/bin/python
from os import walk
import json

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

mypath = "../src"
print("JSON for scl:")
ss = dir2jsonstr(mypath, 'scl', 1)
print(ss)
f = open('scl.json','w')
f.write(ss)
f.close()

print("\n\n"+"JSON for scl_ext:")
ss = dir2jsonstr(mypath, 'scl_ext', 1)
print(ss)
f = open('scl_ext.json','w')
f.write(ss)
f.close()

print("\n\n"+"JSON for scl and scl_ext:")
ss = "{\"name\": \"\",\"children\":["
ss = ss + dir2jsonstr(mypath, 'scl', 1) + ","
ss = ss + dir2jsonstr(mypath, 'scl_ext', 1)
ss = ss + "]}"
print(ss)
f = open('src.json','w')
f.write(ss)
f.close()


print("\n\n"+"JSON for scl and scl_ext without files:")
ss = "{\"name\": \"\",\"children\":["
ss = ss + dir2jsonstr(mypath, 'scl', 0) + ","
ss = ss + dir2jsonstr(mypath, 'scl_ext', 0)
ss = ss + "]}"
print(ss)
f = open('src_only_dirs.json','w')
f.write(ss)
f.close()

