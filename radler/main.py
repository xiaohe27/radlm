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
from radler.radlr.ros import utils, msg, node, packagexml, cmakeliststxt

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
radlr_semantics = Semantics(language)

########
# Parse
########
qname = infos.root_namespace.qualify(source.stem)  # @UndefinedVariable
with source.open() as f:
    infos.ast = radlr_semantics(f.read(), qname, infos.root_namespace)

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
utils.gen_dirs(infos.ast)
msg_list = msg.gen(infos.ast)
gened_cpp_files = node.gen(infos.ast)
packagexml.gen(infos.ast)
cmakeliststxt.gen(msg_list, gened_cpp_files, infos.ast)
