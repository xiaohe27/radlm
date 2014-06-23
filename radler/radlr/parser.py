'''
Created on Apr 2014

@author: léonard Gérard leonard.gerard@sri.com

The semantics is a function (a parser) defined by the given language
description following the meta grammar.
The program parameter given when called (__call__) is written in this
language.
The output is composed of
    - 'types': a mapping between type names and 
'''

from radler.astutils.names import NonExistingIdent, QualifiedName
from radler.astutils.nodeutils import clean_node, ParseVisitor, spprint_node
from radler.astutils.tools import BucketDict, str
import parsimonious
from parsimonious.exceptions import IncompleteParseError
from parsimonious.grammar import Grammar
from radler.radlr import sanitize
from radler.radlr.errors import log1, log_err, log3, log2, error,\
    internal_error
from radler.radlr.metaParser import meta_parser
from radler.radlr.rast import AstNode, Ast, AstVisitor, Ident
from radler.astutils.location import Location
from collections import OrderedDict
from functools import partial

def loc_of_parsimonious(parsimonious_node):
    return Location('', parsimonious_node.full_text,
                        parsimonious_node.start,
                        parsimonious_node.end)

class Env:
    """ Environment associated to a language:
    env.kinds maps kind_name -> 'clas' | 'typ' | 'enum' | 'struct'
    env.keywords is a set of keywords
    """
    def __init__(self, keywords):
        self.kinds = OrderedDict()
        self.keywords = set(keywords)


def gen_grammar(language_tree, params, env):
    # We can update in place kinds, since we do a depth first...
    # not pretty but python dict are easier as mutable variables

    def _unalias(visitor, node, env):
        """ unalias : replace a node with its only child
        """
        (node, env) = visitor.mapred(node, env)
        return node.children[0], env

    symbol = _unalias
    kind = _unalias

    def regex(visitor, node, env):
        """ Keep it as is.
        """
        return node.text, env

    def quoted(visitor, node, env):
        """ A quoted node is a leaf.
        It stores its content in the second child of its choice child.
        """
        return node.children[0][1], env

    def defKind(visitor, node, env):
        """ Verify that the kind is not already defined.
        Return the kind string. The enclosing node has to register k.
        A defKind node has on child the leaf basic_word
        """
        k = node.children[0]
        if k in env.kinds:
            visitor.error("The kind {k} is already defined.".format(k=k))
        return k, env

    def typ(visitor, node, env):
        """ define a kind and construct the grammar.
        A typ node has six childs 'type' defKind 'REGEX' regex 'CXX' quoted
        """
        (node, env) = visitor.mapred(node, env) #depth first
        (_, kind, _, reg, _, _) = node.children
        env.kinds[kind] = 'type' #register this type
        env.keywords.add(kind) #add the type name to the keywords
        g = """{kind} = {kind}_def / _solo_ident
    {kind}_def = _alias_def / {kind}_annoted / {kind}_not_annoted
    {kind}_annoted = (_ident _ ':' _)? '{kind}' _ {kind}_value
    {kind}_not_annoted = (_ident _)? _ {kind}_value
    {kind}_value = {reg} _
            """.format(**locals())
        return g, env


    def some_kind(visitor, node, env):
        """ two childs kind ('/' kind)*
        """
        (node, env) = visitor.mapred(node, env) #depth first
        (k, o) = node.children
        if o:
            kinds = [k]
            for x in o:
                kinds.append(x[1]) #get the other kinds
            g = '({l})'.format(l=' / '.join(kinds))
        else:
            g = k
        return g, env

    def field(visitor, node, env):
        """
        A field node has three childs symbol some_kind ('*' / '+' / '?')?
        Return the grammar string to match it:
        'symbol kind'
        """
        (node, env) = visitor.mapred(node, env) #depth first
        (name, kind, mod) = node.children
        mod = '*' if mod else ''
        g = "'{name}' _ {kind}{mod}".format(name=name, kind=kind, mod=mod)
        env.keywords.add(name) #register the field name as keyword
        return g, env

    def clas(visitor, node, env):
        """ define a kind and construct the grammar.
        A clas node has three childs 'class' defKind field*
        """
        (node, env) = visitor.mapred(node, env) #depth first
        kind = node.children[1]
        env.kinds[kind] = 'class' #register this class
        gfield_list = node.children[2]
        fields = "( {0} )".format(') / ('.join(gfield_list))
        g = """{kind} = {kind}_def / _solo_ident
    {kind}_def = _alias_def / {kind}_annoted / {kind}_not_annoted
    {kind}_annoted = (_ident _ ':' _)? '{kind}' _ {kind}_value
    {kind}_not_annoted = (_ident _)? _ {kind}_value
    {kind}_value = '{{' _ ({fields})* '}}' _
            """.format(**locals())
        return g, env

    def meta(visitor, node, env):
        """Concatenate the grammar strings and add the external definitions
        A meta node has only one real child (clas/type)*
        """
        (node, env) = visitor.mapred(node, env) #depth first
        keywords = '|'.join(env.keywords)
        rules = '\n'.join(node.children[0])
        g = r"""{rules}
            _ident = ~r"(?!({keywords})\b)(?!{forbidden_prefix})[a-zA-Z][a-zA-Z0-9_]*"
            _solo_ident = _ident _ !(':' / '{{' / '=')
            _alias_def = _ident _ '=' _ _solo_ident
            _ = ~r"\s*(#[^\r\n]*\s*)*\s*"
            _end = ~r"$"
            """.format(rules=rules, keywords=keywords,
                       forbidden_prefix=visitor['forbidden_prefix'])
        return g, env

    gen = ParseVisitor(locals(), params=params)
    g, env = gen.visit(language_tree, env)
    if env.kinds:
        topleveldefs = '_def / '.join(env.kinds) + '_def'
    else:
        topleveldefs = ''
    lang = """_lang = _ ({tops})* _""".format(tops=topleveldefs)
    log3(lambda:g+lang)
    return Grammar(g+lang, '_lang'), env

def gen_tree_to_ast(language_tree, env):
    #We create a visitor with a visitor... so the convention here is that
    #mvisitor is the meta visitor which is using menv as accumulator, while
    #visitor is the generated one using ast as accumulator.
    #menv is a dictionnary storing the generated visitor methods.

    def typ(mvisitor, mnode, menv):
        """ A typ node has six childs
            'type' defKind 'REGEX' regex 'CXX' quoted
        It generates one rule named kind from defkind
        """
        #No need to do recursive traversal (no rules are generated lower),
        #    but go fetch the kind in defKind.
        kind = mnode.children[1].children[0]

        def m_def_gen(value_child_nb):
            def m_def(visitor, node, namespace):
                childs, _ = visitor.mapacc(node.children, namespace)
                loc = loc_of_parsimonious(node)
                ident = childs[0][0] if childs[0] else None
                if isinstance(ident, parsimonious.nodes.Node):
                    qname = namespace.qualify(ident.text)
                else:
                    qname = namespace.generate("_"+kind)
                value = childs[value_child_nb].match.group('value')
                n = AstNode(kind, qname, [value], namespace, loc)
                namespace.associate(qname, n)
                return n, namespace
            return m_def

        #basic rules are simply replaced by their lone child
        menv[kind] = ParseVisitor.left_mapacc
        menv[kind+'_def'] = ParseVisitor.left_mapacc
        menv[kind+'_value'] = ParseVisitor.left_mapacc
        menv[kind+'_annoted'] = m_def_gen(2)
        menv[kind+'_not_annoted'] = m_def_gen(1)
        return (), menv

    def field(mvisitor, mnode, menv):
        """ A field node has three childs symbol some_kind ('*' / '+' / '?')?
        Return the tuple (symbol, mod)
        """
        (name, _, mod) = mnode.children
        return (name, mod), env

    def clas(mvisitor, mnode, menv):
        """ A clas node has three childs 'class' defKind field*
        Three rules needs to be dealt with, kind, name_def, name_annoted
        with kind the defKind kind.
        """
#         mnode, menv = mvisitor.node_mapacc(mnode, menv)
        kind = mnode.children[1].children[0]
        field_specs = []
        for field in mnode.children[2]:
            mod = field.children[2]
            fname = field.children[0].text
            field_specs.append((fname, mod))

        def m_def_gen(fields_child_nb):
            def m_def(visitor, node, namespace):
                ident = node.children[0][0] if node.children[0] else None
                if isinstance(ident, parsimonious.nodes.Node):
                    qname = namespace.qualify(ident.text)
                else: #generate a name since none is given
                    qname = namespace.generate("_"+kind)
                thisnamespace = namespace.push(qname)
                childs, _ = visitor.mapacc(node.children, thisnamespace)
                loc = loc_of_parsimonious(node)
                fields = BucketDict(childs[fields_child_nb])
                for (fname, mod) in field_specs:
                    v = fields.get(fname, False)
                    err = lambda m: error(m.format(fname), loc)
                    if not mod:
                        if v is False:
                            err("field {} is mandatory.")
                        elif isinstance(v, list):
                            err("field {} requires one and only one value.")
                    elif mod == '?':
                        if v is False:
                            fields[fname] = None
                        elif len(v) != 1:
                            err("field {} may be given at most one value.")
                        else:
                            fields[fname] = v[0]
                    elif mod == '*':
                        if v is False:
                            fields[fname] = []
                    elif mod == '+':
                        if v is False:
                            err("field {} requires at least one value.")
                    else:
                        internal_error("unknown modifier")
                n = AstNode(kind, qname, fields, thisnamespace, loc)
                namespace.associate(qname, n)
                return n, namespace
            return m_def

        #basic rules are simply replaced by their lone child
        menv[kind] = ParseVisitor.left_mapacc
        menv[kind+'_def'] = ParseVisitor.left_mapacc
        menv[kind+'_value'] = partial(ParseVisitor.left_mapacc, el_num=1)
        menv[kind+'_annoted'] = m_def_gen(2)
        menv[kind+'_not_annoted'] = m_def_gen(1)
        return (), menv

    def meta(mvisitor, mnode, menv):
        _ = mvisitor.mapacc(mnode.children, menv)

        def _lang(visitor, node, namespace):
            #depth first, get a list of definitions
            defs, _ = ParseVisitor.left_mapacc(visitor, node, namespace)
            return (loc_of_parsimonious(node), defs), namespace

        def _alias_def(visitor, node, namespace):
            #depth first, return three childs, ident '=' ident
            loc = loc_of_parsimonious(node)
            ((alias, _, target), _) = visitor.mapacc(node.children, namespace)
            qname = namespace.qualify(alias.text)
            n = AstNode('_alias', qname, [target], namespace, loc)
            namespace.associate(qname, n)
            return n, namespace

        #get the _ident from _solo_ident
        menv['_solo_ident'] = ParseVisitor.left_mapacc
        menv['_lang'] = _lang
        menv['_alias_def'] = _alias_def
        gen = ParseVisitor(menv)
        return gen, menv

    # generate the visitor and return it.
    metagen = ParseVisitor(locals())
    menv = dict()
    gen, menv = metagen.visit(language_tree, menv)

    # resolve idents.
    # We are working on an ast with some leafs being parse nodes (_ident).
    def onleaf(visitor, leaf, namespace):
        if isinstance(leaf, parsimonious.nodes.Node):
            if leaf.expr_name != '_ident':
                raise Exception("The Ast have parsing node "
                                "{} left over".format(leaf.expr_name))
            try:
                qname = namespace.resolve(leaf.text)
            except NonExistingIdent:
                raise Exception("{l}Undefined identifier {t}"
                    "".format(l=str(loc_of_parsimonious(leaf)), t=qname))
            return Ident(qname, loc_of_parsimonious(leaf)), namespace
        else:
            return leaf, namespace
    def onnode(visitor, node, namespace):
        node, _ = visitor.node_mapacc(node, node._namespace)
        return node, namespace
    resolver = AstVisitor(onleaf=onleaf, default=onnode)

    def tree_to_ast(program_tree, program_qname, namespace):
        thisnamespace = namespace.push(program_qname)
        (location, defs), _ = gen.visit(program_tree, thisnamespace)
        ast = Ast(program_qname, location, env.kinds,
                  env.keywords, thisnamespace, defs)
        namespace.associate(program_qname, ast)
        ast, _ = resolver.visit(ast, namespace)
        return ast
    return tree_to_ast



class TypingError(Exception):
    def __init__(self, expected, actual):
        self.expected = expected
        self.actual = actual
    def __str__(self):
        return "expected a {e}, but got a {a}".format(e=str(self.expected),
                                                      a=str(self.actual))
class MissingField(Exception):
    def __init__(self, missing):
        self.missing = missing
    def __str__(self):
        return "missing field {f}".format(f=self.missing)
class DuplicateField(Exception):
    def __init__(self, field):
        self.field = field
    def __str__(self):
        return "the field {f} may only be defined once".format(f=self.field)

def gen_ast_checker(language_tree):
    """ The ast_checker has to:

            Check type of identifiers.

            We want to catch first errors in the higher levels:
                we check types before doing the depth recursion.
    """
    #TODO 5 : finish this thing.

    def clas(mvisitor, mnode, menv):
        kind = mnode.children[1].children[0]
        def m_name(visitor, node, expected_type):
            if kind != expected_type:
                raise TypingError(expected_type, kind)
            return visitor.mapacc(node, expected_type)
        menv[kind] = m_name
        return mnode, menv
    #generate the visitor and return it.
    metagen = ParseVisitor(locals())
    _, menv = metagen.visit(language_tree, dict())
    gen = ParseVisitor(menv)
    def ast_checker(ast):
        ast, _ = gen.visit(ast, ())
        return ast
    return ast_checker



class Semantics:
    def __init__(self, language):
        """
        #TODO: 7 allow to extend a semantics by providing it to another one
        """
        language_tree = meta_parser(language.defs)
        self.env = Env(language.extra_keywords)
        params = {'forbidden_prefix' : getattr(language, 'forbidden_prefix', '')}
        self.grammar, self.env = gen_grammar(language_tree, params, self.env)
        self.tree_to_ast = gen_tree_to_ast(language_tree, self.env)
        self.ast_checker = gen_ast_checker(language_tree)

    def __call__(self, program, program_qname, namespace):
        """ The parser
        #TODO: 6 allow to extend the ast
        """
        try:
            program_tree = self.grammar.parse(program)
            program_tree = clean_node(program_tree, to_prune=['_', '_end'],
                                      keep_regex=True)
        except IncompleteParseError as e:
            log1("Incomplete parsed tree is:")
            program_tree = clean_node(e.node, to_prune=['_', '_end'],
                                      keep_regex=True)
            log1(spprint_node(program_tree))
            log_err(str(e))
            exit(-3)
        log3(lambda: spprint_node(program_tree))
        ast = self.tree_to_ast(program_tree, program_qname, namespace)
        ast = sanitize.update_idents(ast, namespace)
        log1(lambda: str(ast))
        self.ast_checker(ast)
        return ast


