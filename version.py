#!/usr/bin/python

import subprocess
import sys

try:
	#call git describe to get version string
	p=subprocess.Popen(["C:\\Program Files (x86)\\Git\\bin\\git.exe","describe","--dirty=-dty","--always","--match=v*.*"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
	#wait for command to complete
	p.wait()
	#get data from command
	out,err=p.communicate()
	#get error
	err=err.decode("utf-8").strip
	#get returncode
	rc=p.returncode;
except:
	#fake return code to indicate an error
	rc=1
	#fake error
	err='Error encountered while running git describe : '+str(sys.exc_info()[0])

#check for success
if rc==0:
	#get version
	ver=out.decode("utf-8").strip()
	#open output file
	f=open("version.c","wt");
	#write include line
	f.write('#include "ARCbus.h"\n\n')
	#write version line
	f.write('const char ARClib_version[]="'+ver+'";\n\n')
	#close file
	f.close()
else:
	#print error 
	print(err,file=sys.stderr)
	#open output file
	f=open("version.c","wt");
	#write include line
	f.write('#include "ARCbus.h"\n\n')
	#write warning line to indicate versioning problems
	f.write('#warning version not found "'+err+'"\n')
	#write version string as unversioned to indicate an error
	f.write('const char ARClib_version[]="unversioned";\n\n')
	#close file
	f.close()
	