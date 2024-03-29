'''
Created on Jun, 2014
@author: Léonard Gérard leonard.gerard@sri.com

    Module used to store global properties.

    Useful to reduce useless passing of the global context,
but also to break circular dependencies since it depends on nothing.
'''

from pathlib import Path
from radler.astutils.names import RootNamespace

script_dir = Path(__file__).absolute().parent.parent
"The radler Path directory."

lib_dir = script_dir / 'lib'
"The radl lib Path directory."

##########
# Global properties concerning everything which is loaded
##########

root_namespace = RootNamespace()
"The global root namespace."

ros_type_of_struct = dict()
"Mapping from types to ROS type names, used to minimize ROS msg creation."

semantics = None
"Semantics object for the language. It is created during bootstrapping."


##########
# Global properties concerning the current source file.
##########

ast = None
"The program ast. Actually set by the parser."

roscppdest_dir = None
"The destination Path directory"

source_file = None
"The source Path file."
