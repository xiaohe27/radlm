'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.tools import write_file
from radler.radlr.rast import AstVisitor
from radler.radlr.ros.rosutils import filepath_in_qn, qn_msgfile, filepath
from radler.astutils.names import QualifiedName

templates = {
'cmakeliststxt': """
cmake_minimum_required(VERSION 2.8.3)
project({namespace})
set(CMAKE_MODULE_PATH ${{CMAKE_MODULE_PATH}}
  {package_dirs}
)
{packages}
find_package(catkin REQUIRED roscpp message_generation)

add_definitions(-DIN_RADL_GENERATED_CONTEXT)

add_message_files(FILES
  {msg_files}
)
generate_messages(DEPENDENCIES)

catkin_package(
 LIBRARIES {namespace}
 CATKIN_DEPENDS roscpp message_runtime
)
include_directories(
  include src ${{catkin_INCLUDE_DIRS}}
)
{executables}
{targetincludes}
{dependencies}
{link_libraries}

"""
,'packages': "find_package({name} REQUIRED {components})"
,'executables': "add_executable({name} {sources})"
,'targetincludes': """
target_include_directories({name}
  PUBLIC
  {dirs}
  {libdirs}
)"""
,'dependencies': "add_dependencies({name} {namespace}_generate_messages_cpp)"
,'link_libraries': """
target_link_libraries({name}
  ${{catkin_LIBRARIES}}
  {libs}
)"""
,'package_dirs': "{dir}"
,'libs': '${{{libname}_LIBRARIES}}'
,'libdirs': '${{{libname}_INCLUDE_DIRS}}'
}

separators = {
'packages': '\n'
,'addexec': '\n'
,'targetinclude': '\n'
,'adddep': '\n'
,'targetll': '\n'
,'package_dirs': '\n  '
,'libs': ' '
,'libdirs': ' '
}


def app(d, s):
    v = templates[s].format(**d)
    if s not in d or not d[s]: d[s] = v
    else: d[s] = separators[s].join((d[s], v))

lib_templates = ['libs', 'libdirs']

def _from_cxx(visitor, cxx, d):
    _, d = visitor.node_mapred(cxx, d)
    for c in cxx['FILENAME']:
        f = cxx._pwd / c._val
        d['sources'].add(str(f))
    d['dirs'].add(str(cxx._pwd))
    for l in cxx['LIB']:
        d['libname'] = l['CMAKE_MODULE']._val
        for t in lib_templates: app(d, t)
    return (), d


nt = ['executables', 'targetincludes', 'dependencies', 'link_libraries']

def _from_node(visitor, node, d):
    d['name'] = node._qname.name()
    #this node generated file
    d['sources'] = {str(d['gened_cpp_files'][node._qname])}

    #gather needed data
    d['dirs'] = set()
    for t in lib_templates: d[t] = ''
    visitor = visitor.update({'cxx_file': _from_cxx,
                              'cxx_class': _from_cxx})
    _, d = visitor.node_mapred(node, d)
    d['dirs'] = '"{}"'.format('" "'.join(d['dirs']))
    d['sources'] = ' '.join(d['sources'])

    for t in nt: app(d, t)
    return (), d

lt = ['packages', 'package_dirs']

def _from_catkinlib(visitor, lib, d):
    d['name'] = lib['CMAKE_MODULE']._val
    d['components'] = ' '.join(c._val for c in lib['COMPONENTS'])
    d['dir'] = str(lib._pwd)
    for t in lt: app(d, t)
    return (), d

_visitor = AstVisitor({'node' : _from_node,
                       'cmake_library': _from_catkinlib})


def gen(msg_list, gened_cpp_files, ast):
    #The Cmakefile waits for msg files relative to the msg dir
    msg_dir = filepath(qn_msgfile(QualifiedName(ast._qname, '', True)))
    msg_files = (str(m.relative_to(msg_dir)) for m in msg_list)
    d = {'namespace'       : ast._qname.name(),
         'msg_files'       : '\n  '.join(msg_files),
         'gened_cpp_files' : gened_cpp_files}
    for t in nt: d[t] = ''
    for t in lt: d[t] = ''
    _visitor.visit(ast, d)
    app(d, 'cmakeliststxt')
    write_file(filepath_in_qn("CMakeLists.txt", ast._qname), d['cmakeliststxt'])


