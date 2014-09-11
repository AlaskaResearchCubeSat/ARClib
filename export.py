#!/usr/bin/python

import shutil
import os
import subprocess
import sys
import time
import re

inputDir="./"
prefix="Z:\Software"
lib=os.path.join(prefix,"lib")
include=os.path.join(prefix,"include")
basename="BUSlib"
gitpath="C:\\Program Files (x86)\\Git\\bin\\git.exe"

#check if there are uncommited changes
p=subprocess.Popen([gitpath,"diff-index","--quiet","HEAD"])
#wait for command to complete
p.wait()
#get returncode
rc=p.returncode
#not sure why but this is needed
print(rc)
#check return code
if rc!=0:
	print("Error : There are uncommitted changes. Commit or stash before exporting")
	exit(rc)

#first build using crossbuild
#find which crossbuild to use
rowleyPath="C:\\Program Files (x86)\\Rowley Associates Limited\\"
#list rowley folder in program files
dirs=os.listdir(rowleyPath);
#initialize variables
path=None
version=None
#search for MSP430 crossworks
for folder in dirs:
	m=re.search("CrossWorks for MSP430 ([0-9\\.])",folder)
	if m is not None:
		#get version tuple
		ver=tuple(map(int,m.group(1)[0].split('.')))
		#check if a version was found
		if version is None or ver>version:
			version=ver
			path=folder

#get bath to crossbuild
crossbuild=os.path.join(rowleyPath,path,'bin','crossbuild.exe')
#run crossbuild
p=subprocess.Popen([crossbuild,'-batch','-config','MSP430 Debug','BUSlib.hzp'])
#wait for command to complete
p.wait()
#check return code
if p.returncode!=0:
	print("Error : project did not build exiting")
	exit(p.returncode)

#generate tag for export
#get time
t=time.localtime()
#generate tag name based on current date/time
tag=time.strftime("Export-%m-%d-%Y_%H%M%S",t);
#generate message
msg=time.strftime("Exported on %m/%d/%Y at %H:%M:%S",t);
#tag release
p=subprocess.Popen([gitpath,"tag","--force","-m="+msg,tag])
#wait for command to complete
p.wait()

if p.returncode!=0:
	print("Error : could not tag export")
	exit(p.returncode)

for folder in ("MSP430 Release","MSP430 Debug","MSP430 Release CDH","MSP430 Debug CDH"):
    outname=basename+"_"+"_".join(folder.split()[1:])+".hza"
    outpath=os.path.join(lib,outname)
    inpath=os.path.join(inputDir,os.path.join(folder,basename+".hza"))
    print("Copying "+inpath+" to "+outpath)
    shutil.copyfile(inpath,outpath)

for file in ("crc.h","ARCbus.h","DMA.h"):
    outpath=os.path.join(include,file)
    inpath=os.path.join(inputDir,file)
    print("Copying "+inpath+" to "+outpath)
    shutil.copyfile(inpath,outpath)


