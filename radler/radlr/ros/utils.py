'''
Created on Jun, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.radlr.errors import error, internal_error
from radler.radlr import infos
from radler.astutils.names import QualifiedName
from radler.astutils.tools import ensure_dir
from pathlib import Path


def qn_topic(qname):
    return qname.qname('/', root='/')

def qn_cpp(qname):
    return qname.qname('::')

def qn_file(qname, sep='/'):
    if len(qname) != 2:
        error("In ros, paths should have exactly one of depth. "
              "Issues with {}.".format(str(qname)))
    return qname.qname(sep)

def qn_srcfile(qname, sep='/src/'):
    return qn_file(qname, '/src/')

def qn_msgfile(qname):
    return qn_file(qname, '/msg/')

def filepath(name):
    return infos.dest_dir / name

def filepath_in_qn(name, qname):
    return filepath(qn_file(QualifiedName(qname, name, True)))

user_file_relativepath = 'src/_user_code'

radllib_relativepath = 'src/_radl_lib'

def gen_dirs(ast):
    prog_qname = ast._qname
    if len(prog_qname) != 1:
        error("In ros, namespace are top level. "
              "Issues with {}.".format(str(prog_qname)), None)
    f = QualifiedName(prog_qname, '', True)
    ensure_dir(filepath(qn_file(f)))
    ensure_dir(filepath(qn_srcfile(f)))
    ensure_dir(filepath(qn_msgfile(f)))
    u = filepath_in_qn(user_file_relativepath, prog_qname)
    if not u.exists():
        u.symlink_to(infos.source_file.parent, True)
    r = filepath_in_qn(radllib_relativepath, prog_qname)
    if not r.exists():
        r.symlink_to(infos.lib_dir, True)
