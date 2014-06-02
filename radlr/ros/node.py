'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Generate one ROS _node.cpp file per node declaration.

'''
from radlr.rast import AstVisitor
from astutils.tools import write_file


_template_node_cpp ="""
#include "radl_lib.h"
#include "ros/ros.h"
#include "{node_h_name}"
{cxx_includes}

namespace {namespace} {{

int main() {{
  int n = 0; //ros requires a reference...
  ros::init(n, NULL, "{name}"); //TODO : SIGINT management ?
  ros::NodeHandle _h;
  ros::Rate _loop_rate({node[RATE]._val});

  // set up publishers
  {set_pub}

  // set up subscribers
  {set_sub}

  {node[CXX][CLASS]._val} _node;
  while (ros::ok()) {{
    //create outgoing structure
    {out_struct} _out;
    {out_fill}

    //combine incoming messages
    {in_struct} _in;
    ros::spinOnce();
    {in_fill}

    _node.step(&_in, &_out);

    //call publishers
    {pub_call}

    _loop_rate.sleep();
  }}
  return 0;
}}
}}
"""

_template_node_h = """
#pragma once

{in_msg_includes}
{out_msg_includes}

namespace {namespace} {{

struct {out_struct} {{
{out_struct_def}
}};

struct {in_struct} {{
{in_struct_def}
}};
}}
"""

_template_in_msg_include = '''#include "{namespace}/{subtopic}.h"
'''
_template_out_msg_include = '''#include "{namespace}/{pubtopic}.h"
'''

_template_out_struct_def ="""{pubtopic}* {pubname};"""

_template_out_fill="""{pubtopic} {pubname};
    _out.{pubname} = &{pubname};"""

_template_pub_call="""{publishername}(_out->{pubname});"""

_template_set_pub="""ros::Publisher {publishername}_ros = _h.advertise<{pubtopic}>("{pubname}", 10);
  {publisherclass}<{pubtopic}> {publishername}({publishername}_ros);"""

_template_in_struct_def="""{subtopic}::ConstPtr {subname};"""

_template_in_fill="""_in.{subname} = {subscribername}.value();"""

_template_init_msg_fill="""{initmsg}.{fieldname} = {fieldval}"""

_template_set_sub="""{subtopic} {initmsg};
  {init_msg_fill}
  {subtopic}::ConstPtr _wrap{initmsg}(&{initmsg});
  {subscriberclass}<{subtopic}> {subscribername}(_wrap{initmsg});
  boost::function<void (const {subtopic}::ConstPtr&)> {subscribername}_func;   // boost still needed by ROS ?
  {subscribername}_func = boost::ref({subscribername});
  ros::Subscriber {subscribername}_ros = _h.subscribe<{subtopic}>("{subname}", 10, {subscribername}_func);"""

def _include_cxx_class(visitor, node, acc):
        _, acc = visitor.node_mapred(node, acc)
        acc.append('#include "' + node['PATH']._val + '"')
        return _, acc
_include_visitor = AstVisitor({'cxx_class' : _include_cxx_class})

def getincludes(node):
    _, paths = _include_visitor.visit(node, [])
    return '\n'.join(paths)


def gennode(visitor, node, acc):
    """ Nodes are not recursive for now """
    ast, src_directory = acc
    namespace = ast._name
    name = node._name
    node_cpp_name = name + '_node.cpp'
    node_cpp_path = src_directory / node_cpp_name
    node_h_name = name + '_node.h'
    node_h_path = src_directory / node_h_name
    cxx_includes = getincludes(node)
    in_struct = '_in_' + name
    out_struct = '_out_' + name
    #Over the publications
    pub_call = ''
    out_fill = ''
    set_pub = ''
    out_struct_def = ''
    out_msg_includes = ''
    for pub in node['PUBLISHES']:
        d = {'namespace'      : namespace,
             'pubname'        : pub._name,
             'publishername'  : '_' + pub._name + '_pub',
             'publisherclass' : pub['PUBLISHER']['CXX']['CLASS']._val,
             'pubtopic'       : pub['TOPIC']._name}
        pub_call += _template_pub_call.format(**d)
        out_fill += _template_out_fill.format(**d)
        set_pub += _template_set_pub.format(**d)
        out_msg_includes += _template_out_msg_include.format(**d)
        out_struct_def += _template_out_struct_def.format(**d)
    #Over the subscribtions
    in_fill = ''
    set_sub = ''
    in_struct_def = ''
    in_msg_includes = ''
    for sub in node['SUBSCRIBED']:
        d = {'namespace'      : namespace,
             'subname'         : sub._name,
             'subscribername'  : '_' + sub._name + '_sub',
             'subscriberclass' : sub['SUBSCRIBER']['CXX']['CLASS']._val,
             'subtopic'        : sub['TOPIC']._name,
             'initmsg' : '_init_' + sub._name}
        init_msg_fill = ''
        for field in sub['TOPIC']['FIELDS']:
            init_msg_fill += _template_init_msg_fill.format(
                                                    fieldname=field._name,
                                                    fieldval=field._val,
                                                    **d)
        d['init_msg_fill'] = init_msg_fill
        in_fill += _template_in_fill.format(**d)
        set_sub += _template_set_sub.format(**d)
        in_msg_includes += _template_in_msg_include.format(**d)
        in_struct_def += _template_in_struct_def.format(**d)
    #generate files
    node_cpp = _template_node_cpp.format(**locals())
    write_file(node_cpp_path, node_cpp)
    node_h = _template_node_h.format(**locals())
    write_file(node_h_path, node_h)
    return (), acc


visitor = AstVisitor({'node' : gennode})

def gen(src_directory, ast):
    visitor.visit(ast, (ast, src_directory))