'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Module used to store global variables.
Especially useful to break circular dependencies since it depends on nothing.
'''

from pathlib import Path
from radler.astutils.names import __RootNamespace

script_dir = Path(__file__).absolute().parent.parent
"The radler Path directory."

lib_dir = script_dir / 'lib'
"The radl lib Path directory."

ast = None
"The program ast. Actually set by the parser."

root_namespace = __RootNamespace('/')
"The global root namespace"

dest_dir = None
"The destination Path directory"

source_file = None
"The source Path file"

ros_type_of_struct = dict()
"A mapping between types and ros type name."