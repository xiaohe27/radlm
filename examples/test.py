#!/usr/bin/env python3.4
'''
Created on Aug, 2014

@author: lgerard

This script tries to test as much as possible the examples.

As a bare minimum, it tries to compile the examples.

All tests need to be documented in the 'tests' list below.
'''



import subprocess
from collections import OrderedDict
from pathlib import Path
import os
import shutil
import sys
from random import randint


class atest:
    """ Class used to describe a test with its file, folder, dependencies, etc.
    """
    def __init__(self, name, folder='', deps=None):
        self.name = name
        self.folder = Path(folder)
        self.deps = deps if deps != None else []
    @classmethod
    def of_name(cls, name, deps=None):
        return cls(name, folder=name, deps=deps)
    @property
    def file(self):
        return self.name + '.radl'
    @property
    def filepath(self):
        return self.folder / self.file


tests = OrderedDict((
    ('test1', atest.of_name('test1')),
    ('test2' , atest.of_name('test2', deps=['test1'])),
    ('test_alias' , atest.of_name('test_alias')),
    ('house_thermo' , atest.of_name('house_thermo'))
))

nb_tests = len(tests)
nd = len(str(nb_tests))

def clean_mkdir(path):
    if Path(path).exists(): shutil.rmtree(path)
    os.mkdir(path)

# Clean the pervasives
pervasives = '../radler/lib/pervasives.radlo'
if Path(pervasives).exists(): os.remove(pervasives)

# Create and cd to a temporary directory
destination = base_dest = 'tmp'
while Path(destination).exists():
    destination = base_dest + str(randint(0, 99999999))
clean_mkdir(destination)
old_dir = os.getcwd()
os.chdir(destination)

# Make a catkin/src dir for catkin_make
clean_mkdir('catkin')
os.mkdir('catkin/src')


#####
# Compile the tests with radler.sh script

radler_errors = []
radler_skipped = []
radler_compiled = []

class Toskip(Exception): pass

cpt = 0
report = "\nReport:\n"
for t in tests.values():
    try:
        #Do not try to compile if some deps aren't met
        for d in t.deps:
            if not d in radler_compiled:
                raise Toskip
        #Deps are met
        cmd = ["../../radler.sh", "--silent", "-c", "--roscpp_dest", "catkin/src"]
        for d in t.deps:
            cmd.append('-O' + d + '.radlo')
        cmd.append('../' + str(t.filepath))
        #Run it
        #print("\nRun {}".format(' '.join(cmd)))
        r = subprocess.call(cmd)
        if r: #We have an error
            radler_errors.append(t.name)
            report += "Failed to compile {}\n".format(t.name)
        else:
            radler_compiled.append(t.name)
    except Toskip:
        radler_skipped.append(t.name)
        report += "Skipped {} because of {}\n".format(t.name, d)
    #Sum up to the user
    cpt +=1
    e = len(radler_errors)
    s = len(radler_skipped)
    c = len(radler_compiled)
    sys.stdout.write('\r{{:>{nd}}}/{{}} compiled with {{:>{nd}}} errors and ({{:>{nd}}} skipped)'.format(nd=nd).format(
                    cpt, nb_tests, e, s))
print(report)


#####
# Try to catkin make all the ROS package at once

os.chdir('catkin')
try:
    r = subprocess.call(['catkin_make'])
    cat_st = 'failed' if r else 'succeeded'
    print("Catkin {} to compile the {} generated ROS module.".format(
            cat_st, len(radler_compiled)))
except FileNotFoundError:
    print("Could not find and run catkin_make.")



# Get out of the temporary directory and delete it
os.chdir(old_dir)
shutil.rmtree(destination)






