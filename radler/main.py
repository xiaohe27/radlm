'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

import argparse
from pathlib import Path

from radler.radlr import crossrefs, pwds, errors, arrays, infos, alias,\
    language
from radler.radlr.errors import log_err
from radler.radlr.parser import Semantics
from radler.radlr.ros import *

########
# Parse arguments
########

parser = argparse.ArgumentParser()
parser.add_argument('--file', help='the RADL description file')
parser.add_argument('--dest', default='src',
    help='the destination directory for generated files')
verbgroup = parser.add_mutually_exclusive_group()
verbgroup.add_argument('--silent', dest='verb', action='store_const', const=-1,
    default=0, help='set verbosity to -1')
verbgroup.add_argument('--verbosity', type=int, dest='verb', default=0,
    help='verbosity level, negative removes warnings, '
         'positive gives additional informations, '
         'level 2 and higher are mostly for debugging purposes.')
verbgroup.add_argument('--verbose', '-v', dest='verb', action='count',
    help='increase verbosity by 1')
parser.add_argument('--warning_as_errors', action='store_true')
parser.add_argument('--continue_when_errors', action='store_true')
#TODO: 8 flags to decide the generation
#parser.add_argument('--gen', action='accumulate', default='ROS')

args = parser.parse_args()

errors.continue_when_errors = args.continue_when_errors
errors.warning_as_errors = args.warning_as_errors
errors.verbosity_level = args.verb

if not args.file:
    import tkinter.filedialog
    tkinter.Tk().withdraw() # Close the root window
    args.file = tkinter.filedialog.askopenfilename()

source = Path(args.file).resolve()
if not source.is_file():
    log_err("The source file {} doesn't exists.".format(source))
    exit(-1)
infos.source_file = source

dest_dir = Path(args.dest)
if not dest_dir.is_dir():
    log_err("The destination directory {} doesn't exists.".format(dest_dir))
    exit(-2)
infos.dest_dir = dest_dir

########
# Bootstrap the semantics from the language definition
########

infos.semantics = Semantics(language)

# #saving the bootsrap is not possible since pickle doesn't pickle functions.
# #maybe by mixing pickling and marshaling of __code__ of the functions
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
rosutils.gen_dirs(infos.ast)
msg_list = rosmsg.gen(infos.ast)
gened_cpp_files = rosnode.gen(infos.ast)
rospackagexml.gen(infos.ast)
roscmake.gen(msg_list, gened_cpp_files, infos.ast)
