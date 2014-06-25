'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Generate one ROS _node.cpp file per node declaration.

'''
from radlr.rast import AstVisitor
from astutils.tools import write_file
from pathlib import Path
from radlr.errors import error, warning, internal_error
from astutils.idents import Ident

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
  ros::Rate _node_rate({period});

  // set up publishers
  {set_pub}

  // set up subscribers
  {set_sub}

  //create outgoing structure filled with the default values
  {out_struct} _out;
  {out_fill}

  {flags_struct} _flags;
  {in_struct} _in;
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

    _node_rate.sleep();
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
"""ros::Publisher {actionname}_ros = _h.advertise<{topic}>("{name}", 2);
  {actionclass}<{topic}> {actionname}({actionname}_ros);"""
, 'in_struct_def'     : "{topic}::ConstPtr {name};"
, 'in_fill'           : "_in.{name} = {actionname}.value();"
, 'init_msg_fill'     : "{initmsg}.{fieldname} = {fieldval};"
, 'set_sub'           :
"""{topic} {initmsg};
  {init_msg_fill}
  {topic}::ConstPtr _wrap{initmsg}(&{initmsg});
  {actionclass}<{topic}> {actionname}(_wrap{initmsg}, {maxlatency}, {pubperiod});
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
              'in_fill'           : '\n    ', 'init_msg_fill'     : '\n  ',
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
        f = '_user_code/' / node._pwd / node['HEADER']._val
        acc.append('#include "' + str(f) + '"')
        return _, acc
_include_visitor = AstVisitor({'cxx_class' : _include_cxx_class})

def getincludes(node):
    _, paths = _include_visitor.visit(node, [])
    return '\n'.join(paths)



def to_nsec(node):
    if node == None:
        nsec = -1
    elif node._kind == 'msec':
        msec = int(node._val)
        nsec = msec * 1000000
    else:
        internal_error("can't compute time from {n}".format(n=node._name))
    return str(nsec)

def to_ros_duration(node):
    nsec = to_nsec(node)
    return "ros::Duration().fromNSec({})".format(nsec)

def to_ros_val(node):
    if isinstance(node, Ident):
        #TODO: 5 instead of inlining values, use variable declarations
        pass
    if node._kind == 'struct':
        return '{{{}}}'.format(', '.join((to_ros_val(v) for v in node['FIELDS'])))
    elif node._kind == 'field_struct':
        return to_ros_val(node['STRUCT'])
    elif node._kind == 'array':
        return '{{{}}}'.format(', '.join((to_ros_val(v) for v in node['VALUES'])))
    else:
        return node._val

def topic_fields(topic):
    if topic._kind == 'topic_of_struct':
        return topic['STRUCT']['FIELDS']
    else:
        return topic['FIELDS']

def topic_type(topic):
    if topic._kind == 'topic_of_struct':
        return topic['STRUCT']._name
    else:
        return topic._name

def gennode(visitor, node, acc):
    """ Nodes are not recursive for now """
    cpps, namespace, dest_directory = acc
    name = node._name

    d = {'namespace'    : namespace,
         'in_struct'    : '_in_' + name,
         'out_struct'   : '_out_' + name,
         'flags_struct' : '_flags_' + name,
         'cxx_includes' : getincludes(node),
         'period'       : to_ros_duration(node['PERIOD'])}

    #Over publications and subscriptions
    pubsub_templates = ['msg_include', 'flags_struct_def']
    for pt in pubsub_templates: d[pt] = ''
    #Over the publications
    pub_templates = ['pub_call', 'out_fill' , 'set_pub',
                     'out_struct_def', 'pub_flags_fill']
    for pt in pub_templates: d[pt] = ''
    for pub in node['PUBLISHES']:
        d.update({'name'        : pub._name,
                  'actionname'  : '_' + pub._name + '_pub',
                  'actionclass' : pub['PUBLISHER']['CXX']['CLASS']._val,
                  'topic'       : topic_type(pub['TOPIC']),
                  'initmsg'     : '_init_' + pub._name})
        d['init_msg_fill'] = ''
        for field in topic_fields(pub['TOPIC']):
            d.update({'fieldname': field._name, 'fieldval': to_ros_val(field)})
            app(d, 'init_msg_fill')
        for f in pub_templates: app(d, f)
        for f in pubsub_templates: app(d, f)

    #Over the subscriptions
    sub_templates = ['in_fill', 'set_sub', 'in_struct_def',
                     'sub_flags_fill', 'gathered_flags']
    for st in sub_templates: d[st] = ''
    for sub in node['SUBSCRIBES']:
        d.update({'name'        : sub._name,
                  'actionname'  : '_' + sub._name + '_sub',
                  'actionclass' : sub['SUBSCRIBER']['CXX']['CLASS']._val,
                  'topic'       : topic_type(sub['TOPIC']),
                  'initmsg'     : '_init_' + sub._name,
                  'maxlatency'  : to_ros_duration(sub['MAXLATENCY'])})
        try:
            pubperiod = sub['TOPIC']._publisher['PERIOD']
        except AttributeError: #no publisher
            warning("Subscription {} won't compute timeout by lack of declared"
                    " publisher.".format(sub._name), sub._location)
            pubperiod = None
        d['pubperiod'] = to_ros_duration(pubperiod)
        d['init_msg_fill'] = ''
        for field in topic_fields(sub['TOPIC']):
            d.update({'fieldname': field._name, 'fieldval': to_ros_val(field)})
            app(d, 'init_msg_fill')
        for f in sub_templates: app(d, f)
        for f in pubsub_templates: app(d, f)
    #generate the header file
    d['name'] = name
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

