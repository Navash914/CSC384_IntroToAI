#Look for #IMPLEMENT tags in this file.
'''
All models need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = futoshiki_csp_model_1(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the Futoshiki puzzle.

1. futoshiki_csp_model_1 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only 
      binary not-equal constraints for both the row and column constraints.

2. futoshiki_csp_model_2 (worth 20/100 marks)
    - A model of a Futoshiki grid built using only n-ary 
      all-different constraints for both the row and column constraints. 

'''
from cspbase import *
import itertools

def futoshiki_csp_model_1(futo_grid):
    n = len(futo_grid)                      # Size of board
    csp = CSP("futoshiki_csp_model_1")      # CSP object
    domain = [*range(1, n+1)]               # Base Domain for unassigned variables

    # Create and add all variables
    var_array = []
    for row in range(n):
        var_array.append([])
        for col in range(n):
            col_board = col * 2                     # Board columns have inequality operators
            name = "var_{0}_{1}".format(row, col)   # Name variable according to its row and column
            if (futo_grid[row][col_board] == 0):    
                dom = domain                        # Unassigned variable, has full domain
            else:
                dom = [futo_grid[row][col_board]]   # Already assigned variable
            var = Variable(name, dom)
            var_array[row].append(var)              # Add variable to var_array
            csp.add_var(var)                        # Add variable to CSP

    # Create and add constraints
    i = 1           # Counter for Constraint Naming
    
    for row in range(n):
        for col in range(n-1):
            col_board = col * 2
            v1 = var_array[row][col]        # Variable for this row, col

            # Binary Row and Column Constraints
            for c in range(col + 1, n):
                v2 = var_array[row][c]
                constraint = Constraint("C" + str(i), [v1, v2])
                i += 1
                constraint.add_satisfying_tuples(get_diff_sat_tuples(v1.domain(), v2.domain()))
                csp.add_constraint(constraint)

            for r in range(row + 1, n):
                v2 = var_array[r][col]
                constraint = Constraint("C" + str(i), [v1, v2])
                i += 1
                constraint.add_satisfying_tuples(get_diff_sat_tuples(v1.domain(), v2.domain()))
                csp.add_constraint(constraint)

            # Inequality Constraints
            ineq = futo_grid[row][col_board+1]
            if ineq == '.':
                continue

            v2 = var_array[row][col+1]
            if ineq == '<':
                v2, v1 = v1, v2     # Swap variables to interpret as greater than relation
            constraint = Constraint("C" + str(i), [v1, v2])
            constraint.add_satisfying_tuples(get_ineq_sat_tuples(v1.domain(), v2.domain()))
            csp.add_constraint(constraint)

    return (csp, var_array)
    

def futoshiki_csp_model_2(futo_grid):
    n = len(futo_grid)                      # Size of board
    csp = CSP("futoshiki_csp_model_2")      # CSP object
    domain = [*range(1, n+1)]               # Base Domain for unassigned variables

    # Create and add all variables
    var_array = []
    for row in range(n):
        var_array.append([])
        for col in range(n):
            col_board = col * 2                     # Board columns have inequality operators
            name = "var_{0}_{1}".format(row, col)   # Name variable according to its row and column
            if (futo_grid[row][col_board] == 0):    
                dom = domain                        # Unassigned variable, has full domain
            else:
                dom = [futo_grid[row][col_board]]   # Already assigned variable
            var = Variable(name, dom)
            var_array[row].append(var)              # Add variable to var_array
            csp.add_var(var)                        # Add variable to CSP

    # Create and add constraints
    i = 1           # Counter for Constraint Naming

    # ALL-DIFF constraints for row and col
    col_var_array = []          # Array for listing column variables in lists (Essentially transpose of var_array)
    for r in range(n):
        col_var_array.append([])
        for c in range(n):
            col_var_array[r].append(var_array[c][r])
    
    for x in range(n):
        # Row ALL-DIFF
        constraint = Constraint("C" + str(i), var_array[x])
        i += 1
        constraint.add_satisfying_tuples(get_all_diff_sat_tuples(var_array[x]))
        csp.add_constraint(constraint)

        # Column ALL-DIFF
        constraint = Constraint("C" + str(i), col_var_array[x])
        i += 1
        constraint.add_satisfying_tuples(get_all_diff_sat_tuples(col_var_array[x]))
        csp.add_constraint(constraint)
        
    # Add Binary Inequality Constraints
    for row in range(n):
        for col in range(n-1):
            col_board = col * 2
            v1 = var_array[row][col]

            ineq = futo_grid[row][col_board+1]
            if ineq == '.':
                continue

            v2 = var_array[row][col+1]
            if ineq == '<':
                v2, v1 = v1, v2     # Swap variables to interpret as greater than relation
            constraint = Constraint("C" + str(i), [v1, v2])
            constraint.add_satisfying_tuples(get_ineq_sat_tuples(v1.domain(), v2.domain()))
            csp.add_constraint(constraint)

    return (csp, var_array)
    
def get_diff_sat_tuples(domain1, domain2):
    '''
    Returns the not-equal constraint satisfying tuples for two domains
    '''
    ret = []
    for d1 in domain1:
        for d2 in domain2:
            if d1 != d2:
                ret.append((d1, d2))
    return ret

def get_ineq_sat_tuples(domain_greater, domain_lesser):
    '''
    Returns the greater-than constraint satisfying tuples for two domains
    '''
    ret = []
    for dg in domain_greater:
        for dl in domain_lesser:
            if dg > dl:
                ret.append((dg, dl))
    return ret

def get_all_diff_sat_tuples(var_list):
    '''
    Returns the ALL-DIFF constraint satisfying tuples for a list of variables
    '''
    var_doms = []
    for v in var_list:
        var_doms.append(v.domain())
    
    ret = []
    for t in itertools.product(*var_doms):
        sat = True
        for x in range(len(t)-1):
            if (sat is False):
                break
            for y in range(x+1, len(t)):
                if t[x] == t[y]:
                    sat = False
                    break
        
        if sat:
            ret.append(t)

    return ret
