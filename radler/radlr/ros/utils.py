'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''


def qn_topic(qname):
    return qname.qname('/', root='/')

def qn_cpp(qname):
    return qname.qname('::')

def qn_file(qname):
    return qname.qname('/')