'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Generate one ROS _node.cpp file per node declaration.

'''
from radlr.rast import AstVisitor
from astutils.tools import write_file
from pathlib import Path

templates = {
'node_cpp':
"""
/**
/* File Generated by raldr
**/

#include "ros/ros.h"
#include "_radl_lib/radl_roslib.h"
#include "{node_h_name}"
{cxx_includes}

using namespace {namespace};

int main(int argc, const char* argv[]) {{
  int n = 0; //ros requires a reference
  ros::init(n, NULL, "{name}"); //TODO : SIGINT management ?
  ros::NodeHandle _h;
  ros::Rate _loop_rate(ros::Duration({period}));

  // set up publishers
  {set_pub}

  // set up subscribers
  {set_sub}

  //create outgoing structure filled with the default values
  {out_struct} _out;
  {out_fill}

  {flags_struct} _flags;
  {in_struct} _in;

  //Main loop
  ros::Duration _period = ros::Duration({period});
  {node[CXX][CLASS]._val} _node;
  while (ros::ok()) {{
    //combine incoming messages
    ros::spinOnce();
    {in_fill}

    //get and set the flags
    {sub_flags_fill}
    radl::flags_t _gathered_flags = {gathered_flags};
    {pub_flags_fill}

    _node.step(&_in, &_flags, &_out);

    //call publishers
    {pub_call}

    _loop_rate.sleep();
  }}
  return 0;
}}

"""
, 'node_h':
"""
/**
/* File Generated by raldr
**/

#pragma once

#include "_radl_lib/radl_flags.h"

{msg_include}

namespace {namespace} {{

struct {out_struct} {{
  {out_struct_def}
}};

struct {in_struct} {{
  {in_struct_def}
}};

struct {flags_struct} {{
  {flags_struct_def}
}};

}}
"""
, 'msg_include'       : '#include "{namespace}/{topic}.h"'
, 'out_struct_def'    : "{topic}* {name};"
, 'out_fill'          :
"""{topic} {initmsg};
  {init_msg_fill}
  _out.{name} = &{initmsg};"""
, 'pub_call'          : "{actionname}(*_out.{name});"
, 'set_pub'           :
"""ros::Publisher {actionname}_ros = _h.advertise<{topic}>("{name}", 10);
  {actionclass}<{topic}> {actionname}({actionname}_ros);"""
, 'in_struct_def'     : "{topic}::ConstPtr {name};"
, 'in_fill'           : "_in.{name} = {actionname}.value();"
, 'init_msg_fill'     : "{initmsg}.{fieldname} = {fieldval};"
, 'set_sub'           :
"""{topic} {initmsg};
  {init_msg_fill}
  {topic}::ConstPtr _wrap{initmsg}(&{initmsg});
  {actionclass}<{topic}, {maxlatency}> {actionname}(_wrap{initmsg});
  boost::function<void (const {topic}::ConstPtr&)> {actionname}_func;   // boost still needed by ROS ?
  {actionname}_func = boost::ref({actionname});
  ros::Subscriber {actionname}_ros = _h.subscribe<{topic}>("{name}", 10, {actionname}_func);"""
, 'sub_flags_fill'    : "_flags.{name} = {actionname}.get_flags();"
, 'pub_flags_fill'    : "_flags.{name} = _gathered_flags;"
, 'flags_struct_def'  : "radl::flags_t {name};"
, 'gathered_flags'    : "_flags.{name}"
}

separators = {'msg_include'       : '\n'    , 'out_struct_def'    : '\n  ',
              'out_fill'          : '\n  '  , 'pub_call'          : '\n  ',
              'set_pub'           : '\n  '  , 'in_struct_def'     : '\n  ',
              'in_fill'           : '\n    ', 'init_msg_fill'     : '\n    ',
              'set_sub'           : '\n  '  , 'sub_flags_fill'    : '\n    ',
              'pub_flags_fill'    : '\n    ', 'flags_struct_def'  : '\n  ',
              'gathered_flags'    : '|'}

def app(d, s):
    v = templates[s].format(**d)
    if s not in d or not d[s]: d[s] = v
    else: d[s] = separators[s].join((d[s], v))

def join(storage):
    for s in storage:
        if isinstance(storage[s], list):
            storage[s] = separators[s].join(storage[s])


def _include_cxx_class(visitor, node, acc):
        _, acc = visitor.node_mapred(node, acc)
        f = ('_user_code/' + node['PATH']._val
                    + '/' + node['FILENAME']._val + '.h')
        acc.append('#include "' + str(f) + '"')
        return _, acc
_include_visitor = AstVisitor({'cxx_class' : _include_cxx_class})

def getincludes(node):
    _, paths = _include_visitor.visit(node, [])
    return '\n'.join(paths)



def to_rostime_pair(node):
    if node._kind == 'msec':
        msec = int(node._val)
        sec, r_msec = divmod(msec, 1000)
        nsec = r_msec * 1000000
    else :
        raise Exception("can't compute rostime from {n}".format(n=node._name))
    return "{sec}, {nsec}".format(sec=sec, nsec=nsec)



def gennode(visitor, node, acc):
    """ Nodes are not recursive for now """
    cpps, namespace, dest_directory = acc
    name = node._name

    d = {'namespace'    : namespace,
         'in_struct'    : '_in_' + name,
         'out_struct'   : '_out_' + name,
         'flags_struct' : '_flags_' + name,
         'cxx_includes' : getincludes(node),
         'period'       : to_rostime_pair(node['PERIOD'])}
    #Over the publications
    for pub in node['PUBLISHES']:
        d.update({'namespace'   : namespace,
                  'name'        : pub._name,
                  'actionname'  : '_' + pub._name + '_pub',
                  'actionclass' : pub['PUBLISHER']['CXX']['CLASS']._val,
                  'topic'       : pub['TOPIC']._name,
                  'initmsg'     : '_init_' + pub._name})
        d['init_msg_fill'] = ''
        for field in pub['TOPIC']['FIELDS']:
            d.update({'fieldname': field._name, 'fieldval': field._val})
            app(d, 'init_msg_fill')
        for f in ('pub_call', 'out_fill' , 'set_pub' , 'msg_include',
                  'out_struct_def' , 'flags_struct_def' , 'pub_flags_fill'):
            app(d, f)
    #Over the subscribtions
    for sub in node['SUBSCRIBES']:
        d.update({'name'        : sub._name,
                  'actionname'  : '_' + sub._name + '_sub',
                  'actionclass' : sub['SUBSCRIBER']['CXX']['CLASS']._val,
                  'topic'       : sub['TOPIC']._name,
                  'initmsg'     : '_init_' + sub._name,
                  'maxlatency'  : to_rostime_pair(sub['MAXLATENCY'])})
        d['init_msg_fill'] = ''
        for field in sub['TOPIC']['FIELDS']:
            d.update({'fieldname': field._name, 'fieldval': field._val})
            app(d, 'init_msg_fill')
        for f in ('in_fill', 'set_sub', 'msg_include', 'in_struct_def',
                  'flags_struct_def', 'sub_flags_fill', 'gathered_flags'):
            app(d, f)
    #generate the header file
    node_h_name = name + '_node.h'
    node_h_path = dest_directory / node_h_name
    node_h = templates['node_h'].format(**d)
    write_file(node_h_path, node_h)
    #generate the cpp file
    node_cpp_name = name + '_node.cpp'
    node_cpp_path = dest_directory / node_cpp_name
    node_cpp = templates['node_cpp'].format(node_h_name=node_h_name,
                                            rate=node['PERIOD']._val,
                                            node=node,
                                             **d)
    write_file(node_cpp_path, node_cpp)
    #register the cpp file
    cpps[name] = node_cpp_name
    return (), (cpps, namespace, dest_directory)


visitor = AstVisitor({'node' : gennode})

def gen(dest_directory, ast):
    namespace = ast._name
    _, (cpps, _, _) = visitor.visit(ast, ({}, namespace, dest_directory))
    return cpps

