#!/usr/bin/env python

import subprocess
import sys
import traceback

#length of minor version string in digits
minor_ver_len=4

try:
	#path to git
	git_str='git'
	#if using windows use full path
	if sys.platform.startswith(('win','cygwin')):
		git_str="C:\\Program Files (x86)\\Git\\bin\\git.exe"
	#call git describe to get version strin
	p=subprocess.Popen([git_str,"describe","--dirty=-dty","--always","--match=v*.*"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
	#wait for command to complete
	p.wait()
	#get data from command
	out,err=p.communicate()
	#get error
	err=err.decode("utf-8").strip
	#get returncode
	rc=p.returncode;
except Exception as e:
	#fake return code to indicate an error
	rc=1
	#get traceback info for error
	lines=traceback.format_exc().splitlines()
	#fake error from traceback (the last line is the actual error
	err='Error encountered while running git describe : '+lines[-1]

#check for success
if rc==0:
	#get version
	ver=out.decode("utf-8").strip()
	#split version
	vers=ver.split('-')
	#check for proper version string
	if vers[0][0]=='v':
		try:
			(major,minor)=vers[0][1:].split('.')
			#get minor version length
			mlen=len(minor)
			#check version length
			if(mlen>minor_ver_len):
				minor='BUS_INVALID_MINOR_VER'
			else:
				#padd minor version to 4 digits
				minor+=(minor_ver_len-mlen)*'0'
		except:
			major='BUS_INVALID_MAJOR_VER'
			minor='BUS_INVALID_MINOR_VER'
	else:
		major='BUS_INVALID_MAJOR_VER'
		minor='BUS_INVALID_MINOR_VER'
	#check for number of commits
	if len(vers)>1 and vers[1].isdigit():
		commits=vers[1]
	else:
		commits='0'
		has_commits=False
	#check for dty suffix
	if vers[-1]=='dty':
		dirty='BUS_VER_DIRTY'
		vhash=vers[-2]
	else:
		dirty='BUS_VER_CLEAN'
		vhash=vers[-1]
	#check version hash
	if vhash[0]=='g':
		vhash=vhash[1:]
	else:
		vhash=""
	#open output file
	f=open("version.c","wt");
	#write include line
	f.write('#include "ARCbus.h"\n\n')
	#write version string line
	f.write('const char ARClib_version[]="'+ver+'";\n\n')
	#write integer versions
	f.write('const BUS_VERSION ARClib_vstruct={'+major+','+minor+','+commits+','+dirty+',"'+vhash+'"};\n')
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
	f.write('#warning version not found "'+err+'"\n\n')
	#write version string as unversioned to indicate an error
	f.write('const char ARClib_version[]="unversioned";\n\n')
	#write integer versions
	f.write('const unsigned short ARClib_major=BUS_INVALID_MAJOR_VER,ARClib_minor=BUS_INVALID_MINOR_VER;\n')
	#write commit number
	f.write('const unsigned short ARClib_commits=0;\n')
	#write hash string
	f.write('const char ARClib_hash[]="invalid";\n')
	#write dirty flag
	f.write('const unsigned short ARClib_dty=BUS_VER_DIRTY;\n')
	#close file
	f.close()
	
