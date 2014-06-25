'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radlr.parser import Semantics
import radlr.language
from radlr.examples import basic_1to1, thermostat, onetopic
from pathlib import Path
from radlr.ros import msg, node, packagexml, cmakeliststxt
from astutils.tools import ensure_dir
from radlr import crossrefs, pwds, errors, arrays, infos
from astutils.idents import Namespace
from radlr.errors import log_err


import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file', help='the RADL description file')
parser.add_argument('--dest', default='src',
    help='the destination directory for generated files')
verbgroup = parser.add_mutually_exclusive_group()
verbgroup.add_argument('--silent', dest='verb', action='store_const', const=-1,
    default=0, help='set verbosity to -1')
verbgroup.add_argument('--verbosity', dest='verb', default=0,
    help='verbosity level, negative removes warnings, '
         'positive gives additional informations, '
         'level 2 and higher are mostly for debugging purposes.')
verbgroup.add_argument('--verbose', '-v', dest='verb', action='count',
    help='increase verbosity by 1')
parser.add_argument('--warning_as_errors', action='store_true')
parser.add_argument('--continue_when_errors', action='store_true')

args = parser.parse_args()

errors.continue_when_errors = args.continue_when_errors
errors.warning_as_errors = args.warning_as_errors
errors.verbosity_level = args.verb

#Bootstrap the semantics from the language definition
radlr_semantics = Semantics(radlr.language)

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
ast = radlr_semantics(source.open().read(), name, global_namespace)

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

#TODO: 1 reenable cross refs to allow publisher period to be given to the subsrcibtion.
# ast = crossrefs.check_and_link_topics_publisher(ast)

#From here, the ast is "frozen" no copies, etc , to allow cross referencing.
crossrefs.add(ast)
pwds.add(ast, user_src_dir)

msg_file_list = msg.gen(msg_dir, ast)
gened_cpp_files = node.gen(src_dir, ast)
packagexml.gen(source, root_dir, ast)
cmakeliststxt.gen(msg_file_list, gened_cpp_files, root_dir, ast)
