'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

import argparse
from pathlib import Path

from radler.radlr import crossrefs, pwds, errors, arrays, infos, alias,\
    language, objectfile
from radler.radlr.errors import log_err, log3
from radler.radlr.parser import Semantics
from radler.radlr.ros import *

########
# Parse arguments
########

p = argparse.ArgumentParser()

p.add_argument('--roscpp_dest', default=None, metavar='DIR', help='generate roscpp files in the DIR')
p.add_argument('--warning_as_errors', action='store_true', help='warnings are treated as errors')
vgrp = p.add_mutually_exclusive_group()
vgrp.add_argument('--silent', dest='verb', action='store_const', const=-1, default=0, help='set verbosity to -1')
vgrp.add_argument('-v', '--verbose', dest='verb', action='count', help='increase verbosity by 1')
vgrp.add_argument('--verbosity', type=int, dest='verb', default=0, help='verbosity level, negative removes warnings, positive gives additional informations, level 2 and higher are mostly for debugging purposes.')
p.add_argument('--continue_kamikaze', action='store_true', help='tries to recover from errors')
#p.add_argument('--force_bootstrap', action='store_true', help='force regeneration of the p')

p.add_argument('-c', action='store_true', help='generate an object file for default to F.radlo')
p.add_argument('-o', '--object_dest', help='specify a path for the generated object file')
p.add_argument('-O', '--object_files', action='append', help='compiled object files to load')
p.add_argument('radl_file', metavar='F.radl', help='the RADL source file defining a module named F')

args = p.parse_args()

errors.continue_when_errors = args.continue_kamikaze
errors.warning_as_errors = args.warning_as_errors
errors.verbosity_level = args.verb


script_calling_dir = Path.cwd()


source = Path(args.radl_file).resolve()
source.stem
if source.suffix != '.radl':
    log_err("The source file needs to have .radl suffix, {} given."
            "".format(str(source)))
    exit(-1)
if not source.is_file():
    log_err("The source file {} doesn't exist.".format(source))
    exit(-1)
infos.source_file = source

if args.object_files:
    object_files = [Path(f).resolve() for f in args.object_files]
else:
    object_files = []

for f in object_files:
    if f.suffix != '.radlo':
        log_err("Object files need to have .radlo suffix, {} given."
                "".format(str(f)))
        exit(-1)
    if not f.is_file():
        log_err("The object file {} doesn't exist.".format(str(f)))
        exit(-1)

if args.roscpp_dest:
    roscpp_dest = Path(args.roscpp_dest)
    if not roscpp_dest.is_dir():
        log_err("The destination directory {} doesn't exist.".format(roscpp_dest))
        exit(-2)
    infos.roscppdest_dir = roscpp_dest
else:
    roscpp_dest = None


########
# Bootstrap the semantics from the language definition if needed.
########

infos.semantics = Semantics(language)

# #saving the bootsrap is not possible since pickle doesn't pickle functions.
# #so... tree_to_ast and ast_checker are not pickable.
# #Maybe by mixing pickling and marshaling of __code__ of the functions
# #is possible, but for now....
# sem_file = infos.script_dir / 'semantics.pickle'
# try:
#     with sem_file.open('rb') as f:
#         infos.semantics = pickle.load(f)
# except Exception:
#     args.force_bootstrap = True
# 
# if args.force_bootstrap or infos.semantics.lang_version != language.version:
#     infos.semantics = Semantics(language)
#     log3('Bootstrapping')
#     with sem_file.open('wb') as f:
#         pickle.dump(infos.semantics, f)

########
# Load object files
########

for f in object_files:
    objectfile.load(f)

########
# Parse
########
qname = infos.root_namespace.qualify(source.stem)  # @UndefinedVariable
with source.open() as f:
    infos.ast = infos.semantics(f.read(), qname, infos.root_namespace)


######## Freeze #######
# From here, the ast is "structurally frozen",
# No new nodes/children are added and nodes keep their address.
# This allow cross referencing, etc.
########

# Checks
arrays.typecheck(infos.ast)

#Transparent alias to forget about them
alias.make_transparent(infos.ast, infos.root_namespace)

# Embedding information in nodes to allow easier manipulation
crossrefs.add(infos.ast)
pwds.add(infos.ast)


# ROS files generation
if roscpp_dest:
    rosutils.gen_dirs(infos.ast)
    msg_list = rosmsg.gen(infos.ast)
    gened_cpp_files = rosnode.gen(infos.ast)
    rospackagexml.gen(infos.ast)
    roscmake.gen(msg_list, gened_cpp_files, infos.ast)

#Object file generation
destobjf = None
if args.object_dest: #Use user given path
    destobjf = script_calling_dir / args.object_dest
elif args.c: #Use default path
    destobjf = script_calling_dir / (source.stem + '.radlo')

if destobjf: #Write the object file
    objectfile.save(infos.ast, destobjf)
