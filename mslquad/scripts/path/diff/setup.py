from distutils.core import setup, Extension


module1 = Extension('diff',
                    include_dirs = ['/usr/local/include','.'],
                    sources = ['diffmodule.c','diff2SttInC.c','diffMath.c','dMath.c'])

setup (name = 'diff',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module1])