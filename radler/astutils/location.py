'''
Created on May, 2014

@author: Léonard Gérard leonard.gerard@sri.com
'''

class Location:
    """ A location is a segment in a file.
    """
    def __init__(self, filename='', filetext='', start=0, end=0):
        self.filename = filename
        self.filetext = filetext
        self.start = start
        self.end = end
        #precompute columns and lines
        self.start_line = filetext.count('\n', 0, start) + 1
        self.start_column = start - filetext.rfind('\n', 0, start) -1
        self.end_line = filetext.count('\n', start, end) + self.start_line
        self.end_column = end - filetext.rfind('\n', 0, end) -1
        self.multiline = self.start_line != self.end_line

    def __str__(self):
        """ Standard printing of location, especially used for error reporting
        It ends with a new line.
        """
        t = self.filetext
        (bl, bc) = (self.start_line, self.start_column)
        (el, ec) = (self.end_line, self.end_column)
        bbc = t.rfind('\n', 0, self.start)+1
        eec = t.find('\n', self.end)
        if self.multiline: # Two+ liner
            bec = t.rfind('\n', 0, self.end) +1
            ebc = t.find('\n', self.start)
            pref_bline = str(bl) + ':'
            bline = pref_bline + t[bbc : ebc]
            pref_eline = str(el) + ':'
            eline = pref_eline + t[bec : eec]
            buline = ' '*(bc+len(pref_bline))+ '^'*(ebc-bbc-bc)
            euline = ' '*(len(pref_bline)) + '^'*(ec)
            inter = '[...]\n' if (el - bl)>2 else ''
            s = ("from line {bl} column {bc} to line {el} column {ec}:\n"
                 "{bline}\n{buline}\n{inter}{eline}\n{euline}\n"
                 "".format(**locals()))
        else: # One liner
            line = t[bbc : eec]
            uline = ' '*(bc) + '^'*(ec-bc)
            s = ("at line {bl}, column {bc}-{ec}:\n"
                 "{line}\n{uline}\n".format(**locals()))
        return s

no_loc = Location()