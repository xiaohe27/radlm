'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

from radler.astutils.tools import write_file
from radler.radlr.rast import AstVisitor


_template_cmakeliststxt = """
cmake_minimum_required(VERSION 2.8.3)
project({namespace})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)
add_definitions(-DIN__RADL__GENERATED__CONTEXT)
add_message_files(
  FILES
  {msg_files}
)
generate_messages(
  DEPENDENCIES
)
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

_template_addexec = "add_executable({name} {sources})"

_template_targetinclude = "target_include_directories({name} PUBLIC {dirs})"

_template_adddep = "add_dependencies({name} {namespace}_generate_messages_cpp)"

_template_targetll = "target_link_libraries({name} ${{catkin_LIBRARIES}})"


def _sources_cxx(visitor, node, sources):
    _, sources = visitor.node_mapred(node, sources)
    for c in node['FILENAME']:
        f = 'src/_user_code' / node._pwd / c._val
        sources.append(str(f))
    return _, sources
_sources_visitor = AstVisitor({'cxx_class' : _sources_cxx,
                               'cxx_file'  : _sources_cxx})


def get_sources(node, gened_cpp_files):
    _, sources = _sources_visitor.visit(node, [])
    sources.append('src/' + str(gened_cpp_files[node._name]))
    return ' '.join(sources)


def _dirs_cxx(visitor, node, dirs):
    _, dirs = visitor.node_mapred(node, dirs)
    dirs.add(str(node._pwd))
    return _, dirs
_dirs_visitor = AstVisitor({'cxx_class' : _dirs_cxx,
                            'cxx_file'  : _dirs_cxx})


def get_dirs(node):
    _, dirs = _dirs_visitor.visit(node, set())
    return '"{}"'.format('" "'.join(dirs))


def _from_node(visitor, node, d):
    """ Nodes are not recursives """
    d['name'] = node._name
    d['sources'] = get_sources(node, d['gened_cpp_files'])
    d['dirs'] = get_dirs(node)
    d['executables'].append(_template_addexec.format(**d))
    d['targetincludes'].append(_template_targetinclude.format(**d))
    d['dependencies'].append(_template_adddep.format(**d))
    d['link_libraries'].append(_template_targetll.format(**d))
    return (), d

_visitor = AstVisitor({'node' : _from_node})


def get_from_nodes(ast, d):
    (d['executables'], d['targetincludes'],
     d['dependencies'], d['link_libraries']) = ([], [], [], [])
    _visitor.visit(ast, d)
    d['executables'] = '\n'.join(d['executables'])
    d['targetincludes'] = '\n'.join(d['targetincludes'])
    d['dependencies'] = '\n'.join(d['dependencies'])
    d['link_libraries'] = '\n'.join(d['link_libraries'])


def gen(msg_file_list, gened_cpp_files, dest_dir, ast):
    d = {'namespace'       : ast._name,
         'msg_files'       : '\n  '.join(msg_file_list),
         'gened_cpp_files' : gened_cpp_files}
    get_from_nodes(ast, d)
    cmakeliststxt = _template_cmakeliststxt.format(**d)
    cmakeliststxt_path = dest_dir / "CMakeLists.txt"
    write_file(cmakeliststxt_path, cmakeliststxt)


