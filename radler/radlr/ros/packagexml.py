'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
from radler.astutils.tools import write_file
from radler.radlr import infos
from radler.radlr.ros.utils import filepath_in_qn


_template_package_xml = """<?xml version="1.0"?>
<package>
  <name>{namespace}</name>
  <version>0.0.1</version>
  <description>Generated from {source}</description>
  <maintainer email="leonard.gerard@sri.com">Léonard Gérard</maintainer>
  <license>BSD</license>
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <run_depend>roscpp</run_depend>
</package>
"""

def gen(ast):
    package_xml = _template_package_xml.format(namespace=ast._qname,
                                               source=str(infos.source_file))
    write_file(filepath_in_qn("package.xml", ast._qname), package_xml)