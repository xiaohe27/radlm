'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com

Generate one ROS _node.cpp file per node declaration.

'''

from radler.astutils.tools import write_file
from radler.radlr.errors import warning, internal_error
from radler.radlr.rast import AstVisitor, Ident
from radler.radlr.ros.utils import qn_cpp, qn_file, qn_topic, filepath


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

  {out_flags_struct} _out_flags;

  {in_flags_struct} _in_flags;
  const {in_flags_struct} * const _pin_flags = &_in_flags;

  {in_struct} _in;
  const {in_struct} * const _pin = &_in;

  {node[CXX][CLASS]._val} _node;

  while (ros::ok()) {{
    //combine incoming messages
    ros::spinOnce();
    {in_fill}

    //get and set the flags
    {sub_flags_fill}
    radl::flags_t _gathered_flags = {gathered_flags};
    {pub_flags_fill}

    _node.step(_pin, _pin_flags, &_out, &_out_flags);

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

struct {out_struct} {{
  {out_struct_def}
}};

struct {in_struct} {{
  {in_struct_def}
}};

struct {in_flags_struct} {{
  {in_flags_struct_def}
}};

struct {out_flags_struct} {{
  {out_flags_struct_def}
}};

"""
, 'msg_include'       : '#include "{topic_file}.h"'
, 'out_struct_def'    : "{topic_t}* {pubname};"
, 'out_fill'          :
"""{topic_t} {initmsg};
  {init_msg_fill}
  _out.{pubname} = &{initmsg};"""
, 'pub_call'          : "{actionname}(*_out.{pubname});"
, 'set_pub'           :
"""ros::Publisher {actionname}_ros = _h.advertise<{topic_t}>("{topic_name}", 2);
  {actionclass}<{topic_t}> {actionname}({actionname}_ros);"""
, 'in_struct_def'     : "{topic_t}::ConstPtr {subname};"
, 'in_fill'           : "_in.{subname} = {actionname}.value();"
, 'init_msg_fill'     : "{initmsg}.{fieldname} = {fieldval};"
, 'set_sub'           :
"""{topic_t} {initmsg};
  {init_msg_fill}
  {topic_t}::ConstPtr _wrap{initmsg}(&{initmsg});
  {actionclass}<{topic_t}> {actionname}(_wrap{initmsg}, {maxlatency}, {pubperiod});
  boost::function<void (const {topic_t}::ConstPtr&)> {actionname}_func;   // boost still needed by ROS ?
  {actionname}_func = boost::ref({actionname});
  ros::Subscriber {actionname}_ros = _h.subscribe<{topic_t}>("{topic_name}", 10, {actionname}_func);"""
, 'sub_flags_fill'    : "_in_flags.{subname} = {actionname}.get_flags();"
, 'pub_flags_fill'    : "_out_flags.{pubname} = _gathered_flags;"
, 'in_flags_struct_def'  : "radl::flags_t {subname};"
, 'out_flags_struct_def'  : "radl::flags_t {pubname};"
, 'gathered_flags'    : "_in_flags.{subname}"
}

separators = {'msg_include'       : '\n'    , 'out_struct_def'    : '\n  ',
              'out_fill'          : '\n  '  , 'pub_call'          : '\n  ',
              'set_pub'           : '\n  '  , 'in_struct_def'     : '\n  ',
              'in_fill'           : '\n    ', 'init_msg_fill'     : '\n  ',
              'set_sub'           : '\n  '  , 'sub_flags_fill'    : '\n    ',
              'pub_flags_fill'    : '\n    ', 'in_flags_struct_def'  : '\n  ',
              'gathered_flags'    : '|'     , 'out_flags_struct_def'  : '\n  '}

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
        internal_error("can't compute time from {}".format(str(node._qname)))
    return str(nsec)

def to_ros_duration(node):
    nsec = to_nsec(node)
    return "ros::Duration().fromNSec({})".format(nsec)

def to_ros_val(node):
    if isinstance(node, Ident):
        #TODO: 5 instead of inlining values, use variable declarations
        pass
    if node._kind == 'struct':
        return '{{{}}}'.format(', '.join(map(to_ros_val, node['FIELDS'])))
    elif node._kind == 'field_struct':
        return to_ros_val(node['STRUCT'])
    elif node._kind == 'array':
        return '{{{}}}'.format(', '.join(map(to_ros_val, node['VALUES'])))
    else:
        return node._val


def gennode(visitor, node, cpps):
    """ Nodes are not recursive for now """

    d = {'in_struct'       : '_in_t',
         'out_struct'      : '_out_t',
         'in_flags_struct' : '_in_flags_t',
         'out_flags_struct': '_out_flags_t',
         'cxx_includes'    : getincludes(node),
         'period'          : to_ros_duration(node['PERIOD'])}

    #Over publications and subscriptions
    pubsub_templates = ['msg_include']
    for pt in pubsub_templates: d[pt] = ''
    #Over the publications
    pub_templates = ['pub_call', 'out_fill' , 'set_pub', 'out_flags_struct_def',
                     'out_struct_def', 'pub_flags_fill']
    for pt in pub_templates: d[pt] = ''
    for pub in node['PUBLISHES']:
        d.update({'pubname'     : pub._qname.name(),
                  'topic_name'  : qn_topic(pub._qname),
                  'actionname'  : '_' + pub._qname.name() + '_pub',
                  'actionclass' : pub['PUBLISHER']['CXX']['CLASS']._val,
                  'topic_file'  : qn_file(pub['TOPIC']._ros_msg_typename),
                  'topic_t'     : qn_cpp(pub['TOPIC']._ros_msg_typename),
                  'initmsg'     : '_init_' + pub._qname.name()})
        d['init_msg_fill'] = ''
        for field in pub['TOPIC']['FIELDS']:
            d.update({'fieldname': field._qname.name(),
                      'fieldval': to_ros_val(field)})
            app(d, 'init_msg_fill')
        for f in pub_templates: app(d, f)
        for f in pubsub_templates: app(d, f)

    #Over the subscriptions
    sub_templates = ['in_fill', 'set_sub', 'in_struct_def', 'in_flags_struct_def',
                     'sub_flags_fill', 'gathered_flags']
    for st in sub_templates: d[st] = ''
    d['gathered_flags'] = '0'
    for sub in node['SUBSCRIBES']:
        d.update({'subname'     : sub._qname.name(),
                  'topic_name'  : qn_topic(sub._qname),
                  'actionname'  : '_' + sub._qname.name() + '_sub',
                  'actionclass' : sub['SUBSCRIBER']['CXX']['CLASS']._val,
                  'topic_file'  : qn_file(sub['TOPIC']._ros_msg_typename),
                  'topic_t'     : qn_cpp(sub['TOPIC']._ros_msg_typename),
                  'initmsg'     : '_init_' + sub._qname.name(),
                  'maxlatency'  : to_ros_duration(sub['MAXLATENCY'])})
        try:
            pubperiod = sub['TOPIC']._publisher['PERIOD']
        except AttributeError: #no publisher
            warning("Subscription {} won't compute timeout by lack of declared"
                    " publisher.".format(str(sub._qname)), sub._location)
            pubperiod = None
        d['pubperiod'] = to_ros_duration(pubperiod)
        d['init_msg_fill'] = ''
        for field in sub['TOPIC']['FIELDS']:
            d.update({'fieldname': field._qname.name(),
                      'fieldval': to_ros_val(field)})
            app(d, 'init_msg_fill')
        for f in sub_templates: app(d, f)
        for f in pubsub_templates: app(d, f)
    #generate the header file
    qname = node._qname
    d['name'] = str(qname)
    node_h_name = qn_file(node._qname) + '_node.h'
    node_h = templates['node_h'].format(**d)
    write_file(filepath(node_h_name), node_h)
    #generate the cpp file
    node_cpp_name = qn_file(node._qname) + '_node.cpp'
    node_cpp = templates['node_cpp'].format(node_h_name=node_h_name,
                                            rate=node['PERIOD']._val,
                                            node=node,
                                             **d)
    write_file(filepath(node_cpp_name), node_cpp)
    #register the cpp file
    cpps[qname] = node_cpp_name
    return (), cpps


visitor = AstVisitor({'node' : gennode})

def gen(ast):
    _, cpps = visitor.visit(ast, {})
    return cpps
