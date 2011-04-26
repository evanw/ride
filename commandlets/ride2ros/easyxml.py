import xml.dom.minidom
import re

class EasyXML:
    '''
    EasyXML is an easy and concise way to generate XML output in Python.
    It uses a custom attribute getter so element names can be specified
    directly in the code:

      books = EasyXML('books')
      books.book(title='Example A')
      books.book.author(name='John Smith', age=57)
      books.book.publisher(name='Publisher A')
      books.book(title='Example B')
      books.book.author(name='Jane Doe', age=30)
      books.book.author(name='James Cutter', age=45)
      books.book.publisher(name='Publisher B')
      print str(books)

    The above code produces the following XML:

      <books>
        <book title="Example A">
          <author age="57" name="John Smith"/>
          <publisher name="Publisher A"/>
        </book>
        <book title="Example B">
          <author age="30" name="Jane Doe"/>
          <author age="45" name="James Cutter"/>
          <publisher name="Publisher B"/>
        </book>
      </books>

    You don't actually need to create every parent element, allowing the
    following code to work:

      root = EasyXML('root')
      root.a.b.c()
      root.a.b.c()
      root.a()
      root.a.b.c()
      root.a.b.c()
      print str(root)

    The above code produces the following XML:

      <root>
        <a>
          <b>
            <c/>
            <c/>
          </b>
        </a>
        <a>
          <b>
            <c/>
            <c/>
          </b>
        </a>
      </root>

    Creating an element returns that element, which can then be passed
    to helper methods:

      def material(primitive, ambient, diffuse):
          primitive.ambient(r=ambient[0], g=ambient[1], b=ambient[2])
          primitive.diffuse(r=diffuse[0], g=diffuse[1], b=diffuse[2])

      root = EasyXML('root')
      material(root.primitive(type='sphere'), (64, 0, 0), (192, 0, 0))
      material(root.primitive(type='cube'), (0, 64, 0), (0, 192, 0))
      print str(root)

    The above code produces the following XML:

      <root>
        <primitive type="sphere">
          <ambient b="0" g="0" r="64"/>
          <diffuse b="0" g="0" r="192"/>
        </primitive>
        <primitive type="cube">
          <ambient b="0" g="64" r="0"/>
          <diffuse b="0" g="192" r="0"/>
        </primitive>
      </root>
    '''

    def __init__(self, name):
        '''
        Construct a new EasyXML node with a certain name.  This should
        only be used to create the root node, all child nodes should be
        created with the attribute syntax (see books example above).
        '''
        self._parent = None
        self._name = name
        self._elements = []
        self._attributes = {}
        self._element_map = {}

    def __getattr__(self, name):
        '''
        If an element with the given name has already been added, just
        return that element.  Otherwise, return a new element with the
        given name and this object as a parent.  This does NOT add the
        returned element to this object yet, you still need to call the
        returned element to add it.
        '''
        if name.startswith('_'):
            return object.__getattr__(self, name)
        if name in self._element_map:
            return self._element_map[name]
        element = EasyXML(name)
        element._parent = self
        return element

    def __call__(self, **kwargs):
        '''
        Add a new element with our name to our parent element.  Any keyword
        arguments are set as attributes on the new element.  This actually
        adds new elements as far up the parent chain as needed, so you don't
        need to create every parent explicitly.
        '''
        e = new_element = EasyXML(self._name)
        e._parent = self._parent
        e._attributes = kwargs
        while e._parent and e not in e._parent._element_map.values():
            e._parent._elements.append(e)
            e._parent._element_map[e._name] = e
            e = e._parent
        return new_element

    def __str__(self):
        '''
        Return generated XML representing the stored element tree.
        '''
        def to_xml(obj):
            element = doc.createElement(obj._name)
            for k in obj._attributes:
                element.setAttribute(k.replace('_', '-'), str(obj._attributes[k]))
            for e in obj._elements:
                element.appendChild(to_xml(e))
            return element

        doc = xml.dom.minidom.Document()
        doc.appendChild(to_xml(self))
        return re.sub(r'<\?xml.*\?>', '', doc.toprettyxml(indent='  ')).strip()
