============================
ReStructuredText Style Guide
============================

This page describes how reStructuredText (reST) is written for MSL
documentation with examples. The
:ref:`last section documents our formatting conventions
<contributing/rst_style_guide:RestructuredText Formatting Conventions>`.

For more general guides to writing reStructuredText, see Sphinx's 
`reStructuredText Primer <http://sphinx-doc.org/rest.html#explicit-markup>`_
and the `docutils Quick reStructuredText guide
<http://docutils.sourceforge.net/docs/user/rst/quickref.html>`_.

Inline Text Styling
===================

Italics
   ``*italic text*`` → *italic text*.

Bold
   ``**bold text**`` → **bold text**.

Monospace
   ````monospace text```` → ``monospace text``.
   When referring to code objects, it's better to use markup that links to the
   object's API documentation (see the :ref:`rst-code-link` section).

Inline math
   ``:math:`\sqrt{16}``` → :math:`\sqrt{16}` (See also the
   :ref:`contributing/rst_style_guide:Math` section).

.. note::

   Inline styles can't be nested

   For example, you *can't* write ``*see :ref:`this page <label>`*``.

   Inline markup also needs to be surrounded by white space, though trailing
   punctuation is fine.
   You can get around this with an *escaped space* that is otherwise invisible,
   For example ``one\ *word*`` renders as one\ *word*.

Other semantic markup
---------------------

In addition to the fundamental inline typesetting styles above, you may use
additional reST roles to provide semantic meaning to the text. The
documentation's CSS takes advantage of this semantic meaning to provide
visual cues to readers.

Abbreviations
   ``:abbr:`MSL (Multi-robot Systems Lab)``` →
   :abbr:`MSL (Multi-robot Systems Lab)` (a tool tip exposes the definition)

Filenames and paths
   ``:file:`repos.yaml``` → :file:`repos.yaml`

Shell commands
   ``:command:`git rebase -i master``` → :command:`git rebase -i master`

..
  FIXME need to clarify that this is actual for linking, no semantics
  Environment variables
     ``:envvar:`EUPS_PATH``` → :envvar:`EUPS_PATH`

User interface labels
   ``:guilabel:`New Pull Request``` → :guilabel:`New Pull Request`. This markup
   can be used for button labels, menus, or even text labels in interactive
   shell programs.

Keyboard commands
   ``:kbd:`Control-a s``` → :kbd:`Control-a s`. Spell out the keys rather than
   using Emacs short hand, such as ``C-x``.

To semantically markup Python code objects, refer to the section on
:ref:`rst-code-link`.

Lists
=====

Unordered lists can be written as:

.. code-block:: rst

   - First item

     Second paragraph for first item, needs to be consistently indented.
   - Second item

   - You can put spaces between items, or not.

   - Hierarchical lists are also possible

     - Put a blank space before the sub-list
     - And indent the sub-list consistently

   - Last item.

which renders as:

- First item

  Second paragraph for first item, needs to be consistently indented.
- Second item

- You can put spaces between items, or not.

- Hierarchical lists are also possible

  - Put a blank space before the sub-list
  - And indent the sub-list consistently

- Last item.

There should be a blank line before and after the list to separate the list
from paragraphs. Blanks lines are allowed *between* list items as well.

Enumerated lists can be written similarly:

.. code-block:: rst

   1. First thing
   2. Second thing

   or automatically enumerated,

   #. First thing
   #. Second thing

which renders as:

1. First thing
2. Second thing

or automatically enumerated,

#. First thing
#. Second thing

Definition lists
----------------

Definition lists are terms with an indented content section.
For example:

.. code-block:: rst

   MSL
      Multi-robot Systems Lab

produces

MSL
   Multi-robot Systems Lab

Definition lists are not limited to dictionary-like usage; they can be
employed whenever a series of terms with associated micro content is needed.

Sections
========

We create section hierarchies as follows:

.. code-block:: rst

   ==========
   Page Title
   ==========

   First Section
   =============

   First Subsection
   ----------------

   First Sub-subsection
   ^^^^^^^^^^^^^^^^^^^^

   First Sub-sub-subsection
   """"""""""""""""""""""""

   Second Section
   ==============

This specific sequence of section markup styles is not mandated by the reST
specification, but we encourage you to use it for consistency across all
reST documents.

Linking
=======

External links
--------------

Links to external web pages can be made two ways.
The first way is:

.. code-block:: rst

   When writing Python, it's a good idea to use the `PEP8 style guide`_.

   .. _PEP8 style guide: https://www.python.org/dev/peps/pep-0008/

The link reference should be provided directly following the paragraph to
make it easier for editors to ensure the text in backticks matches the link
reference line. Despite this fragility, this style is good since it makes
the reST itself more readable.

The second method is to put the URL inline:

.. code-block:: rst

   When writing Python, it's a good idea to use the
   `PEP8 style guide <https://www.python.org/dev/peps/pep-0008/>`_.

You may decide to use either method, taking readability into consideration.

Either option will be rendered as follows:

When writing Python, it's a good idea to use the `PEP8 style guide
<https://www.python.org/dev/peps/pep-0008/>`_.

.. _rst-internal-links:

Internal links to labels
------------------------

Any content block can be labeled.
For example, to give a section the label ``making-labels``, we write:

.. code-block:: rst

   .. _making-labels:

   Making Labels
   -------------

   Labels are an empty directive that appear directly before any block, such
   as a section, image, table, or code block.
   Labels start with an underscore, and words in labels are separated by
   hyphens.
   Labels are a **global namespace**, so make them as specific as possible.

With the ``:ref:`` role you can link to a labeled block:

.. code-block:: rst

   For internal links, :ref:`you'll need to make labels <making-labels>`.

You can also make references with ``:ref:`label-name``` and the link text
will automatically be populated with the section title or figure caption,
for example.

Remember that labels are **global** across a Sphinx documentation project.
With labels, you can link to sections in other pages.

We are using the ``sphinx.ext.autosectionlabel`` extension, which will
automatically generate labes for each section for us. The labels are generated
with the pattern of ``name_of_folder/name_of_doc_file:section header text``.
For example, to reference this section in the style guide we would use
``:ref:`contributing/rst_style_guide:Internal links to labels```.

Internal links to other pages
-----------------------------

To link to another page in the same doc project, use the ``:doc:`` role with
the **relative path** to the target ``.rst`` document.

.. code-block:: rst

   See our :doc:`Styleguide <rst_style_guide>` to learn how to write reST docs.

Note how the ``.rst`` extension wasn't included.

Links to equations
------------------

Equations can be linked to using a ``:ref:`` to their label, as described
above. If the equation was numbered by adding a ``:label:`` field to the
math directive itself then that equation can be reference with the ``:eq:``
role. See :ref:`contributing/rst_style_guide:block math` for more information.

.. _rst-code-link:

Links to code objects
---------------------

When describing a code object, you can also link to that object's API
definition using a syntax similar to the ``:ref:`` role used above.

Links to Python objects
^^^^^^^^^^^^^^^^^^^^^^^

Objects can be referenced with these roles:

- ``:py:mod:`package.module``` references a module *or package* with namespace
  ``package.module``.
- ``:py:func:`pkg.mod.function``` references a Python function at namespace
  ``pkg.mod.function``.
  The role's text does not need to include trailing parentheses.
- ``:py:class:`pkg.mod.Class``` to reference a class ``Class`` in ``pkg.mod``.
- ``:py:meth:`pkg.mod.Class.method``` to reference a method ``method`` in class
  ``Class`` in ``pkg.mod``.
- ``:py:attr:`pkg.mod.Class.attribute``` to reference an attribute
  ``attribute`` in class ``Class`` in ``pkg.mod``.
- ``:py:data:`pkg.mod.VARIABLE``` to reference a module-level variable
  ``VARIABLE`` in ``pkg.mod``.
- ``:py:const:`pkg.mod.CONSTANT``` to reference a module-level *constant*
  ``CONSTANT`` in ``pkg.mod``.

Namespace resolution
""""""""""""""""""""

In these examples, the full namespace of each Python object is specified.
In some contexts, Sphinx may be able to identify the reference object without
the full namespace. For example in class docstrings, references to methods
or attributes in the same class can be made by name alone. See the
`Sphinx documentation
<http://sphinx-doc.org/domains.html#cross-referencing-python-objects>`_ for
more details on object resolution.

.. _rst-cpp-links:

Customizing the link text
^^^^^^^^^^^^^^^^^^^^^^^^^

By default the full namespace to the object is shown as the linked text.
To show only the name of the object itself, prefix the namespace with ``~``.
For example:

.. code-block:: rst

   :py:func:`~numpy.sin`

will be rendered as :py:func:`~numpy.sin`

As with the ``:ref:`` role, it is also possible to provide custom link text.
For example:

.. code-block:: rst

   :py:func:`Numpy's sine function <numpy.sin>`

Default domains
^^^^^^^^^^^^^^^

By default, these code referencing roles require a *domain prefix* such as
``py`` or ``cpp`` to specify the language of the object being reference.
This prefix can be omitted when the domain is implicitly set, such as in a
Python docstring.

In a reStructuredText document, the domain can be set via the
``default-domain`` directive. For example, to set the default domain for a reST
document to Python, use

.. code-block:: py

   .. default-domain:: py

When this is set, Python code references can be made more concise, e.g.,
``:func:`numpy.sin```.

See `Sphinx's documentation on Domains`_ for more information about
referencing code objects.

.. _Sphinx's documentation on Domains: http://sphinx-doc.org/domains.html

Tables
======

We recommend that you use the `'grid' reST tables`_ syntax for tables, since
they are more flexible than `'simple' reST tables`_. And although not
necessary, we suggest that you provide a caption using the ``table`` directive
and a label prefixed with "``table-``." For example:

.. _`'simple' rest tables`: http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html#simple-tables
.. _`'grid' rest tables`: https://docutils.sourceforge.io/docs/ref/rst/restructuredtext.html#grid-tables

.. literalinclude:: /_static/examples/rst/basic_grid_table.rst
   :language: rst
   :lines: 3-

produces

.. include:: /_static/examples/rst/basic_grid_table.rst
   :start-line: 2

Note how cells can be joined by omitting the dividing line.
The ``=`` characters divide the header from table content.
Text in the header is set in bold.

You can write tables with multiple header rows, including spans across header
cells:

.. literalinclude:: /_static/examples/rst/multi_header_table.rst
   :language: rst
   :lines: 3-

to get:

.. include:: /_static/examples/rst/multi_header_table.rst
   :start-line: 2

In the simplest cases, tables are not required to have headers, or even be
inside a `table directive`_.

.. literalinclude:: /_static/examples/rst/no_header_table.rst
   :language: rst
   :lines: 3-

produces:

.. include:: /_static/examples/rst/no_header_table.rst
   :start-line: 2

Be sure to leave a blank line before and after the `table directive`_.

.. _`table directive`: https://docutils.sourceforge.io/docs/ref/rst/directives.html#tables

Images and Figures
==================

Plain images
------------

Plain images can be included with the ``image`` directive.

.. literalinclude:: /_static/examples/rst/image.rst
   :language: rst
   :lines: 3-

.. include:: /_static/examples/rst/image.rst
   :start-line: 2

This example shows how an image can by hyperlinked to any URL with the
``target`` field. Internal links, as in the example, must be *relative* to
the reST document; Sphinx does not process URLs in an ``image``\ 's
``target`` field.

The ``image`` directive has `more configurable fields
<http://docutils.sourceforge.net/docs/ref/rst/directives.html#image>`_.
If image sizes need to be manipulated from reST, we recommend using ``scale``
since it is responsive. We hope to provide better support for responsive
image sizing.

Be sure to leave a blank line before and after the ``image`` directive.

Figure directive
----------------

Figures include both an image and a caption.

.. literalinclude:: /_static/examples/rst/figure.rst
   :language: rst
   :lines: 3-

.. include:: /_static/examples/rst/figure.rst
   :start-line: 2

Note that the ``:name:`` field takes the place of a separate
:ref:`label <rst-internal-links>` for hyperlinking.
By convention, these labels should be prefixed with "``fig-``."

Be sure to leave a blank line before and after the ``figure`` directive.

Note on paths to image files
----------------------------

Images are included in the ``_static/`` directory of the git repository for
this documentation project. Sphinx requires image assets to be located in
this ``_static/`` directory in order to properly copy files into the built
website. By using a prefix "``/``" we indicate that a path is relative to
the root of the documentation repository.

Package documentation is hosted in ``docs/`` directories of the git
repositories of individual packages. For such package documentation,
image files should be placed inside a directory in ``docs/source/_static/``
named for the package itself. This nested directory structure is needed to
merge package documentation content into the root documentation build.

Math
====

Sphinx allows you to write math expressions with a LaTeX-like plain text
syntax that will be typeset by `MathJax <https://www.mathjax.org>`_ in the
browser. MathJax supports AMSMath-LaTeX syntax.
`This website by Dr Carol Burns
<http://www.onemathematicalcat.org/MathJaxDocumentation/TeXSyntax.htm>`_
provides a comprehensive listing of available LaTeX syntax in MathJax, along
with examples.

In Sphinx, you can either write *inline* expressions with the ``math`` role,
or *block* elements with the ``math`` directive.

Inline math
-----------

Write inline math expressions with the ``math`` role.
For example, ``:math:`\sigma_\mathrm{mean} = \sigma / \sqrt{N}``` produces
:math:`\sigma_\mathrm{mean} = \sigma / \sqrt{N}`.

Block math
----------

To display math as a block element, use the ``math`` directive (be sure to
leave a blank line before and after the ``math`` directive). For example:

.. literalinclude:: /_static/examples/rst/math.rst
   :language: rst
   :lines: 3-

renders as

.. include:: /_static/examples/rst/math.rst
   :start-line: 2

Referencing equations
^^^^^^^^^^^^^^^^^^^^^

Notice the ``:label:`` field in the previous sample; it both annotates the
equation with a number, and allows the equation to be cross-referenced with
the ``eq`` role; for example ```:eq:`math-sample``` produces
:eq:`math-sample`. Equation references may only be made within the same
reStructuredText page as the original ``math`` directive. See
`the Sphinx docs on Math support
<http://www.sphinx-doc.org/en/stable/ext/math.html>`_ for more information.

Multiple Equations
^^^^^^^^^^^^^^^^^^

Multiple equations can appear in the same ``math`` directive.
Simply include a blank line between each equation (and don't include an
equation as a argument of the ``math`` directive itself).

.. literalinclude:: /_static/examples/rst/math_multi.rst
   :language: rst
   :lines: 3-

renders as

.. include:: /_static/examples/rst/math_multi.rst
   :start-line: 2

Aligned Equations
^^^^^^^^^^^^^^^^^

Often when there are multiple statements in a ``math`` directive it's
desirable to align those statements around the equals sign, for example.
In AMSMath-LaTeX this would be achieved with the ``align`` environment.
In reStructuredText we can accomplish the same in a ``math`` directive.

.. literalinclude:: /_static/examples/rst/math_align.rst
   :language: rst
   :lines: 3-

renders as

.. include:: /_static/examples/rst/math_align.rst
   :start-line: 2

Notice how the alignment point is marked with an ``&`` and ``\\`` is appended
to each math statement *except for the last.* Also note how there are no
blank lines *between* math statements.

Source code
===========

For blocks of code, we prefer the ``code-block`` directive.
This directive has the form

.. code-block:: rst

   .. code-block:: <language>
      :name: optional-label
      :emphasize-lines: <optional lines to highlight>

      <code>

where

- ``<language>`` can be `any token understood by Pygments`_, particularly
  ``py`` (python), ``cpp`` (C++), ``java`` (Java), ``js`` (JavaScript) and
  ``rst`` (reStructuredText). Specify ``none`` to disable highlighting.
- ``:name:`` is an explicit hyperlink label for the code block.
- ``:emphasize-lines:`` is an optional sequence of lines to highlight. This can
  be comma-separated, with hyphens to indicate spans.

.. _any token understood by Pygments: http://pygments.org/docs/lexers/

Be sure to leave a blank line before and after the ``code-block`` directive.

Including source code examples from other files with literalinclude
-------------------------------------------------------------------

The ``code-block`` directive is great for code examples written in the
reStructuredText source file itself. You might also want to show a code
sample contained in a separate file. For this you can use the
``literalinclude`` directive:

.. code-block:: rst

   .. literalinclude:: path/to/example.py
      :language: py

The source path can either be relative to the reST document or relative to
the documentation root by prefixing the path with ``/``.

The ``literalinclude`` directive also supports ``code-block`` fields, such
as ``name`` and ``emphasize-lines``. In addition, you can selective
include ranges of lines with the ``lines`` field. For example, to include
only lines 10 -- 20:

.. code-block:: rst

   .. literalinclude:: path/to/example.py
      :language: py
      :lines: 10-20

To omit the first two lines from a file:

.. code-block:: rst

   .. literalinclude:: path/to/example.py
      :language: py
      :lines: 2-

Sophisticated inclusion patterns can be achieved by listing multiple spans,
such as ``:lines: 3-10,20-``, which shows the first ten lines and all lines
after the 20\ :sup:`th`\ .

When including code example snippets from other files, it may be useful to
remove indentation. Use the ``dedent`` field for that.
For example:

.. code-block:: rst

   .. literalinclude:: path/to/example.py
      :language: py
      :lines: 5-10
      :dedent: 4

will show lines 5 -- 10 and remove 4 space characters (presumably because the
snippet is inside a Python class or function).

Lightweight syntax for Python code and sessions
-----------------------------------------------

You can markup Python code blocks using a lightweight syntax:

.. code-block:: rst

   ::

      print('hello world!')

Interactive python sections can be marked up as

>>> print('Hello world!')
Hello world!

Command line prompts
====================

For generic command line prompts, we use the `sphinx-prompt`_ extension
so that the prompt character (e.g, ``$``) isn't selectable. This is great
for giving copy-and-paste-ready command line instructions.

.. _sphinx-prompt: https://github.com/sbrunner/sphinx-prompt

For basic bash prompts,

.. code-block:: rst

   .. code-block:: shell

      mkdir -p hello/world
      cd hello/world

produces

.. code-block:: shell

   mkdir -p hello/world
   cd hello/world

Footnotes
=========

Footnotes should be used sparingly, if at all, in documentation. Prefer
inline hyperlinks to other sections. If you *do* need footnotes, you can
make them as follows:

.. code-block:: rst

   This is a line.\ [#label]_

   .. [#label] This is the footnote content.

Note that we had to provide an escaped space for the footnote mark occurring
after a period.

The footnote content should occur not far from the inline footnote mark;
generally provide the footnote content at the end of the section.

Citations
=========

Citations should be used for scholarly references; use hyperlinks for web
native content. Citations can be made as follows:

.. code-block:: rst

   The LSST Project [Ivezic2008]_ will produce 15 TB of images per night.

   .. [Ivezic2008] Ivezic et al 2008. *LSST: from Science Drivers to
                   Reference Design and Anticipated Data Products.*
                   `arxiv:0805.2366 <http://arxiv.org/abs/0805.2366>`_

Citations are distinguished from footnotes in that the label *does not*
begin with a ``#``.

Comments in ReStructuredText
============================

Provide comments to fellow writers using ``..``,

.. code-block:: rst

   .. This is a one-line comment.

   ..
      This is the first of a multi-paragraph comment.

      The second paragraph.

Avoid using comments to keep around old or alternate versions of text; prefer
using Git version control instead.

RestructuredText Formatting Conventions
=======================================

Text wrapping
-------------

When writing reST documentation in Python docstrings, documentation lines
should be wrapped at lengths of 79 characters for consistency with
PEP8 Python Style Guide.

.. NOTE: ls.st/rfc-107

For reStructuredText documents (e.g., ``.rst`` files), reST doesn't care
about line formatting. Emacs users, for example, are free to use hard-wrap
formatting lines at 72 characters if that helps you write docs.
Whenever possible, we *encourage* you to use soft-wrapping for your text.
This allows others format text columns in their editors as they wish.
As well, the GitHub.com code editor does not have hard-wrap auto-formatting.
Those making doc edits on GitHub.com will tend to use soft-wrap by default
(see '`GitHub Flow in the Browser`_').

.. _GitHub Flow in the Browser: https://help.github.com/articles/github-flow-in-the-browser/

When using soft-wrap formatting, you might **write one sentence per line**
(i.e., put a line break after each sentence).  As a writer, this has the
advantage of making it easier to check the rhythm of your writing, including
sentence lengths. Shorter sentences are easier to read.
`One-sentence-per-line`_ is also semantically correct in the sense of Git.

.. _One-sentence-per-line: https://xkcd.com/1285/

Indentation
-----------

ReStructuredText should be indented consistently with the context, which
generally means taking visual alignment cues rather than adhering to a fixed
indent width.

In directives, align to the directive's name:

.. code-block:: rst

   .. code-block:: python

      print('hello world')

In lists, align naturally with the text:

.. code-block:: rst

   - First item.

     Another paragraph for the first item.
   - Second item.

Note how that alignment adapts to numbered lists:

.. code-block:: rst

   1. First item.

      Another paragraph for the first item.
   2. Second item.

For argument lists in Python docstrings, we indent descriptions by four spaces:

.. code-block:: rst

   Parameters
   ----------
   x_coord : float
       Particle's x-coordinate.
   y_coord : float
       Particle's y-coordinate.

.. note::

   Thanks for Astro Data Lab for the inspiration of this content. See
   https://datalab.noao.edu/docs/manual/ for reference.