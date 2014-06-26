'''
Created on Apr, 2014

@author: lgerard
'''
import unittest

from parsimonious.exceptions import ParseError
from radlr.parser import meta_parser


class TestMetaParser(unittest.TestCase):

    def test_meta_keyword(self):
        self.assertRaises(ParseError, meta_parser, "class class")
        self.assertRaises(ParseError, meta_parser, "type class")
        self.assertRaises(ParseError, meta_parser, "enum class")
        self.assertRaises(ParseError, meta_parser, "enum type")

#     def test_comments(self):
#         self.assertEqual(self.parse("class C")(), self.parse("""# commenting
#         
#         #spaces and more
#         class #inline comment
# 
#             C
#         # Haha
#         
#         """)())




if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()