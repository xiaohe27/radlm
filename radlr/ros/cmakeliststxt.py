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
  include ${{catkin_INCLUDE_DIRS}} {dest_dir}
)
{executables}
{dependencies}
{link_libraries}

"""

_template_addexec = "add_executable({name} {sources})"

_template_adddep = "add_dependencies({name} {namespace}_generate_messages_cpp)"

_template_targetll = "target_link_libraries({name} ${{catkin_LIBRARIES}})"


def _sources_cxx_class(visitor, node, acc):
        _, (sources, user_src_dir) = visitor.node_mapred(node, acc)
        f = (user_src_dir / (node['PATH']._val)
                         / (node['FILENAME']._val + '.cpp'))
        sources.append(str(f))
        return _, (sources, user_src_dir)
_sources_visitor = AstVisitor({'cxx_class' : _sources_cxx_class})

def get_sources(node, gened_cpp_files, user_src_dir):
    _, (sources, _) = _sources_visitor.visit(node, ([], user_src_dir))
    sources.append('src/' + str(gened_cpp_files[node._name]))
    return ' '.join(sources)


def _from_node(visitor, node, acc):
    """ Nodes are not recursives """
    execs, deps, lls, gened_cpp_files, user_src_dir, namespace = acc
    n = node._name
    execs.append(_template_addexec.format(
        name=n, sources=get_sources(node, gened_cpp_files, user_src_dir)))
    deps.append(_template_adddep.format(name=n, namespace=namespace))
    lls.append(_template_targetll.format(name=n))
    return (), (execs, deps, lls, gened_cpp_files, user_src_dir, namespace)

_visitor = AstVisitor({'node' : _from_node})

def get_from_nodes(ast, gened_cpp_files, user_src_dir, namespace):
    acc = ([],[],[],gened_cpp_files,user_src_dir,namespace)
    _, (execs, deps, lls, _, _, _) = _visitor.visit(ast, acc)
    return ('\n'.join(execs), '\n'.join(deps), '\n'.join(lls))


def gen(msg_file_list, gened_cpp_files, dest_dir, user_src_dir, ast):
    namespace = ast._name
    msg_files = '\n  '.join(msg_file_list)
    executables, dependencies, link_libraries = (
        get_from_nodes(ast, gened_cpp_files, user_src_dir, namespace))
    cmakeliststxt = _template_cmakeliststxt.format(**locals())
    cmakeliststxt_path = dest_dir / "CMakeLists.txt"
    write_file(cmakeliststxt_path, cmakeliststxt)


