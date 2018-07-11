from distutils.core import setup, Extension

# Distributed SE3 without compensation
module1 = Extension('cvxse3',
                    include_dirs = ['/usr/local/include', '.', './cvxgen_se3/'],
					sources = ['se3_module.c', './cvxgen_se3/solver.c', './cvxgen_se3/matrix_support.c', 
							   './cvxgen_se3/ldl.c', './cvxgen_se3/util.c'])

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
                    include_dirs = ['/usr/local/include', '.', './cvxgen_se3_comp/'],
					sources = ['se3_comp_module.c', './cvxgen_se3_comp/solver.c', './cvxgen_se3_comp/matrix_support.c', 
							   './cvxgen_se3_comp/ldl.c', './cvxgen_se3_comp/util.c'])

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

# Trajectory
module3 = Extension('cvxtraj',
                    include_dirs = ['/usr/local/include', '.', './cvxgen_traj/'],
					sources = ['traj_module.c', './cvxgen_traj/solver.c', './cvxgen_traj/matrix_support.c', 
							   './cvxgen_traj/ldl.c', './cvxgen_traj/util.c'])

setup (name = 'cvxtraj',
       version = '0.1',
       description = 'TBD',
       author = 'Zijian Wang',
       author_email = 'zjwang@stanford.edu',
       url = 'https://docs.python.org/extending/building',
       long_description = '''
Long description TBD
''',
       ext_modules = [module3])