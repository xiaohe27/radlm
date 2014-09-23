#!/usr/bin/env python3.4
'''
Created on Aug, 2014

@author: Léonard Gérard leonard.gerard@sri.com

This script tries to test as much as possible the examples.

As a bare minimum, it tries to compile the examples.

All tests need to be documented in the tests section below.
'''



from collections import OrderedDict
import os
from pathlib import Path
from random import randint
import shutil
import subprocess
import sys

from tarjan import tarjan


def call(*args, **kargs):
    """Call a subprocess with subprocess.call but setup the pwd env var first.
    """
    os.putenv('PWD', os.getcwd())
    return subprocess.call(*args, **kargs)


class Atest:
    """ Class used to describe a test with its file, folder, dependencies, etc.
    """
    def __init__(self, file, failing, folder='.', deps=None):
        if file[0:1].isnumeric():
            print("ERROR, tests can't begin with a num (for mangling)")
            exit(-1)
        self.file = file
        self.failing = failing
        self.folder = Path(folder)
        self.deps = deps if deps != None else []
    @property
    def filepath(self):
        return self.folder / self.file
    @property
    def objfile(self):
        p = '_'.join(self.folder.parts)
        # Mangling is correct because names don't begin with numbers.
        return str(len(p)) + p + str(len(self.file)) + self.file + '.radlo'

class Good(Atest):
    """ Atest with failing set to False,
    file computed from name with radl extension
    folder appended with 'good/'
    """
    def __init__(self, name, folder='', deps=None):
        Atest.__init__(self, name+'.radl', False, 'good/'+folder, deps)

class GoodAlone(Good):
    """ Good test in its own folder with the same name. """
    def __init__(self, name, deps=None):
        Good.__init__(self, name, name, deps)

class Bad(Atest):
    """ Atest with failing set to True """
    def __init__(self, name, folder='', deps=None):
        Atest.__init__(self, name+'.radl', True, 'bad/'+folder, deps)



#-------------------------
# Declare tests here
#-------------------------


test1 = Good('test1')
test2 = Good('test2', deps=[test1])
test_alias = Good('test_alias')
test3 = Good('test3', deps=[test_alias])
topic_dec = Good('topic_dec')
topic_struct = Good('topic_struct')
topic_struct1 = Good('topic_struct1')
type_sizes = Good('type_sizes')
static_libraries = Good('static_libraries')
house_thermo = GoodAlone('house_thermo')
thermostat = GoodAlone('thermostat')


b_int8 = Bad('int8')
b_int8__129 = Bad('int8_-129')
b_int8_128 = Bad('int8_128')
b_uint8__1 = Bad('uint8_-1')
b_uint8_257 = Bad('uint8_256')


#-------------------------
# End of test declarations
#-------------------------



# Tarjan the tests, the result is an ordering of the cliques.

tests = dict((t, t.deps) for t in locals().values() if isinstance(t, Atest))
tj_tests = tarjan(tests)

# Flatten and error when circular dep of tests (a clique of more than one elt).

tests = []
for i in range(len(tj_tests)):
    if len(tj_tests[i]) != 1:
        print("ERROR, the tests {} have a circular dependency."
              "".format(str(tj_tests[i])))
        exit(-1)
    else:
        tests.append(tj_tests[i][0])

# Now tests is correctly sorted and ready to be used.

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

log_file = Path()
i = 0
while log_file.exists():
    log_file = Path('../test' + str(i) + '.log')
    i += 1

log_file_d = log_file.open('w')
def log(msg):
    print("\n@@@@\n@@@@ "+msg+"\n@@@@", file=log_file_d, flush=True)

cpt = 0
report = ""
for t in tests:
    try:
        #Do not try to compile if some deps aren't met
        for d in t.deps:
            if not d in radler_compiled:
                raise Toskip
        #Deps are met
        cmd = ["../../radler.sh",
               "-o", t.objfile,
               "--roscpp_dest", "catkin/src"]
        for d in t.deps:
            cmd.append('-O' + d.objfile)
        cmd.append('../' + str(t.filepath))
        #Run it
        log("Test {} ({})".format(t.filepath, ' '.join(cmd)))
        r = call(cmd, stdout=log_file_d)
        if r and not t.failing: #We have an error
            radler_errors.append(t)
            msg = "\nFailed to compile {}".format(t.filepath)
            report += msg
            log(msg)
        elif not r and t.failing: #We should have failed
            radler_errors.append(t)
            msg = "\n{} Should have failed to compile".format(t.filepath)
            report += msg
            log(msg)
        else:
            if t.failing:
                log("Successfully failed.")
            else:
                log("Successfully compiled.")
            radler_compiled.append(t)
    except Toskip:
        radler_skipped.append(t)
        report += "\nSkipped {} because of {}".format(t.filepath, d.filepath)
    #Sum up to the user
    cpt +=1
    e = len(radler_errors)
    s = len(radler_skipped)
    c = len(radler_compiled)
    sys.stdout.write('\r{{:>{nd}}}/{{}} compiled with {{:>{nd}}} errors and ({{:>{nd}}} skipped)'.format(nd=nd).format(
                    cpt, nb_tests, e, s))
print()
if report:
    print("Report (see {} for full log):".format(log_file.name)+report)



#####
# Try to catkin make all the ROS package at once

os.chdir('catkin')
try:
    r = call(['catkin_make'])
    cat_st = 'failed' if r else 'succeeded'
    print("Catkin {} to compile the {} generated ROS module.".format(
            cat_st, len(radler_compiled)))
except FileNotFoundError:
    print("Could not find and run catkin_make.")



# Get out of the temporary directory and delete it
os.chdir(old_dir)
shutil.rmtree(destination)






