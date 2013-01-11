#!/usr/bin/python

import shutil
import os

inputDir="./"
outDir=os.path.join(inputDir,"Library")

for folder in ("MSP430 Release","MSP430 Debug","MSP430 Release CDH","MSP430 Debug CDH"):
    outname="BUSlib_"+"_".join(folder.split()[1:])+".hza"
    outpath=os.path.join(outDir,outname)
    inpath=os.path.join(inputDir,os.path.join(folder,"BUSlib.hza"))
    print("Copying "+inpath+" to "+outpath)
    shutil.copyfile(inpath,outpath)

for file in ("crc.h","ARCbus.h"):
    outpath=os.path.join(outDir,file)
    inpath=os.path.join(inputDir,file)
    print("Copying "+inpath+" to "+outpath)
    shutil.copyfile(inpath,outpath)

