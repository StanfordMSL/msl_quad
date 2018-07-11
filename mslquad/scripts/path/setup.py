from distutils.core import setup, Extension

# Distributed SE3 without compensation
module1 = Extension('cvxse3',
                    include_dirs = ['/usr/local/include', './cvxgen/', './cvxgen/cvxgen_se3/'],
					sources = ['./cvxgen/se3_module.c', './cvxgen/cvxgen_se3/solver.c', 
                               './cvxgen/cvxgen_se3/matrix_support.c', 
							   './cvxgen/cvxgen_se3/ldl.c', './cvxgen/cvxgen_se3/util.c'])

setup (name = 'cvxse3',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module1])

# With compensation
module2 = Extension('cvxse3comp',
                    include_dirs = ['/usr/local/include', './cvxgen', './cvxgen/cvxgen_se3_comp/'],
					sources = ['./cvxgen/se3_comp_module.c', './cvxgen/cvxgen_se3_comp/solver.c', 
                               './cvxgen/cvxgen_se3_comp/matrix_support.c', 
							   './cvxgen/cvxgen_se3_comp/ldl.c', './cvxgen/cvxgen_se3_comp/util.c'])

setup (name = 'cvxse3comp',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module2])

# differential flatness
module3 = Extension('diff',
                    include_dirs = ['/usr/local/include', './diff'],
                    sources = ['./diff/diffmodule.c','./diff/diff2SttInC.c','./diff/diffMath.c','./diff/dMath.c'])

setup (name = 'diff',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang, Dingjiang Zhou',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module3])

# cvxgen trajectory
module4 = Extension('cvxtraj',
                    include_dirs = ['/usr/local/include', './cvxgen', './cvxgen/cvxgen_traj/'],
					sources = ['./cvxgen/traj_module.c', './cvxgen/cvxgen_traj/solver.c', 
                               './cvxgen/cvxgen_traj/matrix_support.c', 
							   './cvxgen/cvxgen_traj/ldl.c', './cvxgen/cvxgen_traj/util.c'])

setup (name = 'cvxtraj',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module4])