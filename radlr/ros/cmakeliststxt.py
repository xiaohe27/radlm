'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from pathlib import Path
from radlr.rast import AstVisitor
from astutils.tools import write_file

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
  include src ${{catkin_INCLUDE_DIRS}}
)
{executables}
{dependencies}
{link_libraries}

"""

_template_addexec = "add_executable({name} {sources})"

_template_adddep = "add_dependencies({name} {namespace}_generate_messages_cpp)"

_template_targetll = "target_link_libraries({name} ${{catkin_LIBRARIES}})"


def _sources_cxx_class(visitor, node, acc):
        _, sources = visitor.node_mapred(node, acc)
        f = ('src/user_code/' + node['PATH']._val
                    + '/' + node['FILENAME']._val + '.cpp')
        sources.append(str(f))
        return _, sources
_sources_visitor = AstVisitor({'cxx_class' : _sources_cxx_class})

def get_sources(node, gened_cpp_files):
    _, sources = _sources_visitor.visit(node, [])
    sources.append('src/' + str(gened_cpp_files[node._name]))
    return ' '.join(sources)


def _from_node(visitor, node, acc):
    """ Nodes are not recursives """
    execs, deps, lls, gened_cpp_files, namespace = acc
    n = node._name
    execs.append(_template_addexec.format(
        name=n, sources=get_sources(node, gened_cpp_files)))
    deps.append(_template_adddep.format(name=n, namespace=namespace))
    lls.append(_template_targetll.format(name=n))
    return (), (execs, deps, lls, gened_cpp_files, namespace)

_visitor = AstVisitor({'node' : _from_node})

def get_from_nodes(ast, gened_cpp_files, namespace):
    acc = ([], [], [], gened_cpp_files, namespace)
    _, (execs, deps, lls, _, _) = _visitor.visit(ast, acc)
    return ('\n'.join(execs), '\n'.join(deps), '\n'.join(lls))


def gen(msg_file_list, gened_cpp_files, dest_dir, ast):
    namespace = ast._name
    msg_files = '\n  '.join(msg_file_list)
    executables, dependencies, link_libraries = (
        get_from_nodes(ast, gened_cpp_files, namespace))
    cmakeliststxt = _template_cmakeliststxt.format(**locals())
    cmakeliststxt_path = dest_dir / "CMakeLists.txt"
    write_file(cmakeliststxt_path, cmakeliststxt)


