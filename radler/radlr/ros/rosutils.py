'''
Created on Jun, 2014
@author: Léonard Gérard leonard.gerard@sri.com

    Utilities to handle file paths, filenames, etc.
'''
from radler.radlr.errors import error_noloc
from radler.radlr import infos
from radler.astutils.names import QualifiedName
from radler.astutils.tools import ensure_dir
from pathlib import Path


def qn_topic(qname):
    return qname.qname('/', root='/')

def qn_msgtype(qname):
    return qname.qname('/')

def qn_cpp(qname):
    return qname.qname('::')

def qn_file(qname, sep='/', prefix='', suffix=''):
    if len(qname) != 2:
        error_noloc("In ROS, paths should have exactly one of depth. "
              "Issues with {}.".format(str(qname)))
    return qname.qname(sep, prefix=prefix, suffix=suffix)

def qn_dir(qname):
    if len(qname) != 1:
        error_noloc("In ROS, paths should have exactly one of depth. "
              "Issues with {}.".format(str(qname)))
    return Path(qname.name())

def qn_srcfile(qname, prefix='', suffix=''):
    return qn_file(qname, '/src/', prefix, suffix)

def qn_msgfile(qname, prefix='', suffix=''):
    return qn_file(qname, '/msg/', prefix, suffix)

def filepath(name):
    return infos.roscppdest_dir / name

user_file_relativepath = Path('src/user_src')

radllib_relativepath = Path('src/radllib')

def gen_dirs(ast):
    prog_qname = ast._qname
    if len(prog_qname) != 1:
        error_noloc("In ros, namespace are top level. "
              "Issues with {}.".format(str(prog_qname)))
    f = QualifiedName(prog_qname, '', True)
    ensure_dir(filepath(qn_file(f)))
    ensure_dir(filepath(qn_srcfile(f)))
    ensure_dir(filepath(qn_msgfile(f)))
    u = filepath(qn_dir(prog_qname) / user_file_relativepath)
    if not u.exists():
        u.symlink_to(infos.source_file.parent, True)  # @UndefinedVariable
    r = filepath(qn_dir(prog_qname) / radllib_relativepath)
    if not r.exists():
        r.symlink_to(infos.lib_dir, True)
