'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Module used to store global variables.
Especially useful to break circular dependencies since it depends on nothing.
'''

ast = None
"The program ast. Actually set by the parser."


ros_type_of_struct = dict()
"A mapping between type and ros type name."