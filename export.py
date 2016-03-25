#!/usr/bin/python

import shutil
import os
import subprocess
import sys
import time
import re
import argparse

inputDir=os.path.dirname(os.path.realpath(sys.argv[0]))
default_prefix=os.path.realpath(os.path.join(inputDir,"../../"))

parser = argparse.ArgumentParser(description='Export library files to export path')
parser.add_argument('-H','--only-headers',dest='headers',action='store_true',help='only export headers')
parser.add_argument('-p','--prefix',action='store',dest='prefix',default=default_prefix,help='output directory prefix')
				  
#Parse command line arguments
args = parser.parse_args()

lib=os.path.join(args.prefix,"lib")
include=os.path.join(args.prefix,"include")
basename="BUSlib"
gitpath="C:\\Program Files (x86)\\Git\\bin\\git.exe"

#check if there are uncommited changes
rc=subprocess.call([gitpath,"-C",inputDir,"diff-index","--quiet","HEAD"])
#check return code
if rc!=0:
	subprocess.call([gitpath,"-C",inputDir,"status"])
	print("Error : There are uncommitted changes. Commit or stash before exporting")
	exit(rc)

#if only exporting headers skip build and library export
if not args.headers:
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



	for config in ("MSP430 Release","MSP430 Debug","MSP430 Release CDH","MSP430 Debug CDH"):

		#build using crossbuild
		print("Building "+config);
		rc=subprocess.call([crossbuild,'-config',config,basename+'.hzp'])
		#check return code
		if rc!=0:
			print("Error : project did not build exiting")
			exit(rc)

		outname=basename+"_"+"_".join(config.split()[1:])+".hza"
		outpath=os.path.join(lib,outname)
		inpath=os.path.join(inputDir,os.path.join(basename+" "+config,basename+".hza"))
		print("Copying "+inpath+" to "+outpath)
		shutil.copyfile(inpath,outpath)

	#generate tag for export
	#get time
	t=time.localtime()
	#generate tag name based on current date/time
	tag=time.strftime("Export-%m-%d-%Y_%H%M%S",t);
	#generate message
	msg=time.strftime("Exported on %m/%d/%Y at %H:%M:%S",t);
	#tag release
	rc=subprocess.call([gitpath,"-C",inputDir,"tag","--force","-m="+msg,tag])

	if rc!=0:
		print("Error : could not tag export")
		exit(rc)
		
	
for file in ("crc.h","ARCbus.h","DMA.h"):
    outpath=os.path.join(include,file)
    inpath=os.path.join(inputDir,file)
    print("Copying "+inpath+" to "+outpath)
    shutil.copyfile(inpath,outpath)

print("Export Completed Successfully!");
