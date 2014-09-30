'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.names import QualifiedName
from radler.astutils.tools import write_file
from radler.radlr.errors import internal_error
from radler.radlr.rast import AstVisitor
from radler.radlr.ros import rosutils
from radler.radlr.ros.rosutils import qn_dir, qn_msgfile, filepath, \
    qn_dir


templates = {
'cmakeliststxt': """
cmake_minimum_required(VERSION 2.8.3)
project({namespace})
set(CMAKE_MODULE_PATH ${{CMAKE_MODULE_PATH}}
  {package_dirs}
)
{packages}
find_package(catkin REQUIRED roscpp message_generation)
{static_libs}


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

install(TARGETS
  {node_to_install}
  {lib_to_install}
  ARCHIVE DESTINATION ${{CATKIN_PACKAGE_LIB_DESTINATION}}
  LIBRARY DESTINATION ${{CATKIN_PACKAGE_LIB_DESTINATION}}
  RUNTIME DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}}
)

"""
,'packages': "find_package({name} REQUIRED {components})"
,'executables': "add_executable({name} {sources})"
,'node_to_install':"{name}"
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
,'lib_to_install': '{name}'
,'static_libs': """
add_library({name} STATIC
  {sources})
set({name}_LIBRARIES {name})
set({name}_INCLUDE_DIRS {headers})
"""
}

separators = {
'packages': '\n'
,'executables' : '\n'
,'targetincludes': '\n'
,'dependencies': '\n'
,'link_libraries': '\n'
,'package_dirs': '\n  '
,'libs': ' '
,'libdirs': ' '
,'static_libs': '\n'
,'sources': '\n  '
,'node_to_install': '\n  '
,'lib_to_install': '\n  '
}


def app(d, s):
    v = templates[s].format(**d)
    if s not in d or not d[s]: d[s] = v
    else: d[s] = separators[s].join((d[s], v))

lib_templates = ['libs', 'libdirs']

def _from_cxx(visitor, cxx, d):
    _, d = visitor.node_mapred(cxx, d)
    #give the correct root to relative paths
    pwd = rosutils.user_file_relativepath / cxx._pwd
    d['dirs'].add(pwd)
    for c in cxx['FILENAME']:
        d['sources'].add(pwd / c._val)
    for l in cxx['LIB']:
        if l._kind == 'cmake_library':
            d['libname'] = l['CMAKE_MODULE']._val
        elif l._kind == 'static_library':
            d['libname'] = l._qname.name()
        else:
            internal_error("unknown library type")
        for t in lib_templates: app(d, t)
    return (), d


nt = ['executables', 'targetincludes', 'dependencies', 'link_libraries',
      'node_to_install']

def _from_node(visitor, node, d):
    d['name'] = node._qname.name()
    #this node generated file, makes it correctly relative to the cmakefile
    srcfile = d['gened_cpp_files'][node._qname].relative_to(d['localroot'])
    d['sources'] = {str(srcfile)}

    #gather needed data
    d['dirs'] = set()
    for t in lib_templates: d[t] = ''
    visitor = visitor.update({'cxx_file': _from_cxx,
                              'cxx_class': _from_cxx})
    _, d = visitor.node_mapred(node, d)
    d['dirs'] = '"{}"'.format('" "'.join(map(str,d['dirs'])))
    d['sources'] = ' '.join(map(str,d['sources']))

    for t in nt: app(d, t)
    return (), d


lt = ['packages', 'package_dirs']

def _from_catkinlib(visitor, lib, d):
    d['name'] = lib['CMAKE_MODULE']._val
    d['components'] = ' '.join(c._val for c in lib['COMPONENTS'])
    d['dir'] = str('${CMAKE_CURRENT_SOURCE_DIR}'
                   / rosutils.user_file_relativepath
                   /lib._pwd)
    for t in lt: app(d, t)
    return (), d


sl = ['static_libs', 'targetincludes', 'dependencies', 'link_libraries',
      'lib_to_install']

def _from_staticlib(visitor, lib, d):
    d['name'] = lib._qname.name()
    d['sources'] = set()
    d['dirs'] = set()
    for t in lib_templates: d[t] = ''
    _, d = visitor.update({'cxx_file': _from_cxx}).node_mapred(lib, d)
    d['dirs'] = '"{}"'.format('" "'.join(map(str,d['dirs'])))
    d['sources'] = ' '.join(map(str, d['sources']))
    cwd = rosutils.user_file_relativepath / lib._pwd
    d['headers'] = ' '.join(str(cwd / h._val) for h in lib['HEADER_PATHS'])

    for t in sl: app(d, t)
    return (), d


_visitor = AstVisitor({'node' : _from_node,
                       'cmake_library': _from_catkinlib,
                       'static_library': _from_staticlib})


def gen(msg_list, gened_cpp_files, ast):
    #The Cmakefile waits for msg files relative to the msg dir
    msg_dir = filepath(qn_msgfile(QualifiedName(ast._qname, '', True)))
    msg_files = (str(m.relative_to(msg_dir)) for m in msg_list)
    localroot = qn_dir(ast._qname)
    d = {'namespace'       : ast._qname.name(),
         'msg_files'       : '\n  '.join(msg_files),
         'gened_cpp_files' : gened_cpp_files,
         'localroot'       : localroot}
    for t in nt: d[t] = ''
    for t in lt: d[t] = ''
    for t in sl: d[t] = ''
    _visitor.visit(ast, d)
    app(d, 'cmakeliststxt')
    write_file(filepath(localroot / "CMakeLists.txt"), d['cmakeliststxt'])


