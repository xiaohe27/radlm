'''
Created on Apr 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''
import unittest

import pypeg2

from radler.radlr.Oparser import InlineValue, Topic


class Test_InlineValue(unittest.TestCase):

    def setUp(self):
        self.parse = (lambda string: lambda:
                      pypeg2.parse(string, InlineValue))
        self.compose = (lambda string: lambda:
                        pypeg2.compose(string, InlineValue))

    def test_short(self):
        self.assertRaises(SyntaxError, self.parse("a"))
        self.assertRaises(SyntaxError, self.parse("' b"))
        self.assertRaises(SyntaxError, self.parse("'abc'"))
        p = self.parse("'abc")()
        self.assertEqual(p.value, "abc")
        self.assertEqual(str(p), "'abc")
        self.assertEqual(eval(repr(p)), p)
        self.assertEqual(repr(p), repr(self.parse(self.compose(p)())()))

    def test_long(self):
        self.assertRaises(SyntaxError, self.parse("""'<k>abc<kk>"""))
        ls = """><>int>
                abc kk<<int >"""
        fls = "'<int>" + ls + "<int>"
        p = self.parse(fls)()
        self.assertEqual(p.value, ls)
        self.assertEqual(p.key, "int")
        self.assertEqual(str(p), fls)
        self.assertEqual(eval(repr(p)), p)
        self.assertEqual(repr(p), repr(self.parse(self.compose(p)())()))

class Test_Topic(unittest.TestCase):
    def setUp(self):
        self.parse = (lambda string: lambda:
                      pypeg2.parse(string, Topic))
    def test_1(self):
        t1 = """ t1 : TOPIC {
            PERIOD 50
            }
            """
        p = self.parse(t1)()
        self.assertEqual(p.kind, "t1")
        self.assertEqual(p.desc, "TOPIC")
        self.assertEqual(p.PERIOD, 50)
        t2 = """ t2 : TOPIC {
            PACKAGES
                tf : ROS {}
            STATE :
                x : int
                y : float
            PUBLISHES :
            STATE :
                z : int
            }
            """
        p = self.parse(t2)()
        self.assertEqual(p.kind, "t2")
        self.assertEqual(p.STATE, )
            
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.test_ExternalValue']
    unittest.main()