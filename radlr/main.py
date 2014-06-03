'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radlr.parser import meta_parser, Semantics
from radlr.language import radlr_language
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
radlr_grammar = meta_parser(radlr_language)
# pprint_node(radlr_grammar) 
radlr_semantics = Semantics(radlr_language)


# t = radlr_semantics(onetopic.code)
# basic_1to1 = radlr_semantics(basic_1to1.code)
# tast = radlr_semantics(thermostat.code, 'toto')

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('file', help='the RADL description file')
parser.add_argument('--dest', default = 'src',
                    help='the destination directory for generated files')
args = parser.parse_args()

source = Path(args.file).absolute()
source_dir = source.parent
name = source.stem
root_dir = Path(args.dest) / name
ensure_dir(root_dir)

# radl_source = Path('/Users/lgerard/tmp/house_thermo.radl')

ast = radlr_semantics(source.open().read(), name)

msg_dir = root_dir / 'msg'
ensure_dir(msg_dir)
src_dir = root_dir / 'src'
ensure_dir(src_dir)
user_src_dir = src_dir / 'user_code'
user_src_dir.symlink_to(source_dir, True)
script_dir = Path(__file__).absolute().parent
lib_dir = script_dir / 'lib'
(src_dir / 'radl_lib.h').symlink_to(lib_dir / 'radl_lib.h')


msg_file_list = msg.gen(msg_dir, ast)
gened_cpp_files = node.gen(src_dir, ast)
packagexml.gen(source, root_dir, ast)
cmakeliststxt.gen(msg_file_list, gened_cpp_files, root_dir, user_src_dir, ast)