'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from pathlib import Path
from radlr.rast import AstVisitor
from astutils.tools import write_file
from radlr.ros.node import gen_source_node

_template_cmakeliststxt = """
cmake_minimum_required(VERSION 2.8.3)
project({namespace})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)
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
  include ${{catkin_INCLUDE_DIRS}}
)
{executables}
{dependencies}
{link_libraries}

"""

_template_addexec = "add_executable({name} {sources})"

_template_adddep = "add_dependencies({name} {namespace}_generate_messages_cpp)"

_template_targetll = "target_link_libraries({name} ${{catkin_LIBRARIES}})"


def _sources_cxx_class(visitor, node, acc):
        _, acc = visitor.node_mapred(node, acc)
        f = Path(node['PATH']._val)
        f = f / (node['FILENAME']._val + '.cpp')
        acc.append(str(f))
        return _, acc
_sources_visitor = AstVisitor({'cxx_class' : _sources_cxx_class})

def get_sources(node):
    _, sources = _sources_visitor.visit(node, [])
    sources += gen_source_node(node) + '.cpp'
    return ' '.join(sources)


def _from_node(visitor, node, acc):
    """ Nodes are not recursives """
    execs, deps, lls, namespace = acc
    n = node._name 
    execs.append(_template_addexec.format(name=n, sources=get_sources(node)))
    deps.append(_template_adddep.format(name=n, namespace=namespace))
    lls.append(_template_targetll.format(name=n))
    return (), (execs, deps, lls, namespace)

_visitor = AstVisitor({'node' : _from_node})

def get_from_nodes(ast, namespace):
    _, (execs, deps, lls, _) = _visitor.visit(ast, ([], [], [], namespace))
    return ('\n'.join(execs), '\n'.join(deps), '\n'.join(lls))


def gen(msg_file_list, dest_dir, ast):
    namespace = ast._name
    msg_files = '  ' + '\n  '.join(msg_file_list)
    executables, dependencies, link_libraries = get_from_nodes(ast, namespace)
    cmakeliststxt = _template_cmakeliststxt.format(**locals())
    cmakeliststxt_path = dest_dir / "CMakeLists.txt"
    write_file(cmakeliststxt_path, cmakeliststxt)


