'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radlr.parser import meta_parser, Semantics
import radlr.language
from parsimonious.nodeutils import pprint_node
from radlr.examples import basic_1to1, thermostat, onetopic
from pathlib import Path
from radlr.ros import msg, node, packagexml, cmakeliststxt
from astutils.tools import ensure_dir

# test1 = r"""
# class topic
# class node
#     TOPS topic *
# """ 

# test1_grammar = meta_parser(test1)
# pprint_node(test1_grammar)
# 
# radlr_grammar = meta_parser(radlr_language)
# pprint_node(radlr_grammar) 

#Bootstrap the semantics from the language definition
radlr_semantics = Semantics(radlr.language)

# t = radlr_semantics(onetopic.code)
# basic_1to1 = radlr_semantics(basic_1to1.code)
# tast = radlr_semantics(thermostat.code, 'toto')

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file', help='the RADL description file')
parser.add_argument('--dest', default='src',
                    help='the destination directory for generated files')
args = parser.parse_args()

source = Path(args.file)
if not source.is_file():
    print("The source file {} doesn't exists.".format(source))
    exit(-1)
source.absolute().resolve() #TODO 4: is it needed ? how to ensure ~ is correctly expanded
source_dir = source.parent
name = source.stem

dest_dir = Path(args.dest)
if not dest_dir.is_dir():
    print("The destination directory {} doesn't exists.".format(dest_dir))
    exit(-2)
dest_dir.absolute().resolve() #TODO 4: is it needed ? how to ensure ~ is correctly expanded
root_dir = Path(args.dest) / name
ensure_dir(root_dir)

ast = radlr_semantics(source.open().read(), name)

msg_dir = root_dir / 'msg'
ensure_dir(msg_dir)
src_dir = root_dir / 'src'
ensure_dir(src_dir)
user_src_dir = src_dir / 'user_code'
if not user_src_dir.exists():
    user_src_dir.symlink_to(source_dir, True)
radl_lib_file = src_dir / 'radl_lib.h'
if not radl_lib_file.exists():
    script_dir = Path(__file__).absolute().parent
    lib_dir = script_dir / 'lib'
    radl_lib_file.symlink_to(lib_dir / 'radl_lib.h')


msg_file_list = msg.gen(msg_dir, ast)
gened_cpp_files = node.gen(src_dir, ast)
packagexml.gen(source, root_dir, ast)
cmakeliststxt.gen(msg_file_list, gened_cpp_files, root_dir, ast)
