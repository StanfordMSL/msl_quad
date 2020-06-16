# Getting Started With Documentation

Documentation is a big part of adding context and ease of use with our project.
Therefore, we stress documenting your code, features and everything else within
MSL Quad.

## Documenting Code

<!-- TODO: Need to add information about documenting code. DocString style we'll use? Tutorial? Other resources? -->

## Documentation For the Web

We use ReadTheDocs (RTD) to host our documentation. In order to generate the
documentation, we use a collection of ReStructureText (RST) and Markdown (MD)
files to write the content within the documentation you are reading now.

Generation of the actual html is done internally by RTD with every commit. But
before you start modifying the documentation and committing it to the repo, it
is essential to build the documentation locally to verify how it will compile
and look on RTD. We use Sphinx through Python, with extensions for MD and RTD.
Below provides a setup for your system to handle the html generation locally.

### Local Documentation

First we need to make sure Python is on your system. We suggest using version 3
and avoiding version 2 entirely as it is now end of life. With Python on your
system there are two ways we'll discuss to setup your system. Here we show how
to use Virtual Environments in Python, but if you ignore the installation and
activation steps for the Virtual Environment below, you can use the system level
Python installation for generating documentation.

#### Installing Necessary Tools

In a terminal install virtualenv for Python (virtualenv needs to be on the
system level for Python to find it properly, so we need root accces.)

```bash
sudo pip install virtualenv
```

Once installed, create a virtual environment and activate it.

```bash
virtualenv /path/to/virtual_environments/rtd-env
source /path/to/virtual_environments/rtd-env/bin/activate
```

<!-- TODO: How do we actually do notes, warnings, etc. in MD? `.. note:` for RST -->
> NOTE:
  If you would like to make this a bit easier to initialize the next time you
  start your PC, you can add the last command to your bash as an alias.
  `alias rtd-env='source /path/to/virtual_environments/rtd-env/bin/activate`

You should now be in your virtual environment. Let's install Sphinx and RTD
extensions.

```bash
pip install sphinx
pip install sphinx-rtd-theme
pip install recommonmark
```

#### Building Local Documentation

We need to build the documentation locally to verify the look on the official
documentation site. This is pretty straightforward and just takes one line in
a terminal and a browser to view the html.

```bash
cd /path/to/repo/docs
make html
```

That's it!

Now you can view the documentation in a browser by following the path to the
generated html. This should be something like
`file:///path/to/repo/docs/build/html/index.htnl`

> NOTE: If you aren't seeing the changes you've made, it's generally good to
  also run `make clean` before building the documentation. This is generally
  necessary if working with other levels of the documentation.