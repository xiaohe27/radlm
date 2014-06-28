'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

import argparse
from pathlib import Path

from radler.astutils.idents import Namespace
from radler.astutils.tools import ensure_dir
from radler.radlr import crossrefs, pwds, errors, arrays, infos
from radler.radlr.errors import log_err
from radler.radlr.examples import basic_1to1, thermostat, onetopic
from radler.radlr import language
from radler.radlr.parser import Semantics
from radler.radlr.ros import msg, node, packagexml, cmakeliststxt


parser = argparse.ArgumentParser()
parser.add_argument('file', help='the RADL description file')
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

#Bootstrap the semantics from the language definition
radlr_semantics = Semantics(language)

# t = radlr_semantics(onetopic.code)
# basic_1to1 = radlr_semantics(basic_1to1.code)
# tast = radlr_semantics(thermostat.code, 'toto')

source = Path(args.file).resolve() #TODO: 4 pathlib issue with '~'
if not source.is_file():
    log_err("The source file {} doesn't exists.".format(source))
    exit(-1)
source_dir = source.parent
name = source.stem

dest_dir = Path(args.dest)
if not dest_dir.is_dir():
    log_err("The destination directory {} doesn't exists.".format(dest_dir))
    exit(-2)
root_dir = Path(args.dest) / name
ensure_dir(root_dir)


global_namespace = Namespace()

with source.open() as f:
    infos.ast = radlr_semantics(f.read(), name, global_namespace)

msg_dir = root_dir / 'msg'
ensure_dir(msg_dir)
src_dir = root_dir / 'src'
ensure_dir(src_dir)
user_src_dir = src_dir / '_user_code'
if not user_src_dir.exists():
    user_src_dir.symlink_to(source_dir, True)
radllib_link = src_dir / '_radl_lib'
if not radllib_link.exists():
    script_dir = Path(__file__).absolute().parent
    lib_dir = script_dir / 'lib'
    radllib_link.symlink_to(lib_dir, True)


#################
# From here, the ast is "structurally frozen",
# No new nodes/children are added and nodes keep their address.
# This allow cross referencing, etc.
#################

# Checks
arrays.typecheck(infos.ast)

# Embedding information in nodes to allow easier manipulation
crossrefs.add(infos.ast)
pwds.add(infos.ast, user_src_dir)

# ROS files generation
msg_file_list = msg.gen(msg_dir, infos.ast)
gened_cpp_files = node.gen(src_dir, infos.ast)
packagexml.gen(source, root_dir, infos.ast)
cmakeliststxt.gen(msg_file_list, gened_cpp_files, root_dir, infos.ast)
