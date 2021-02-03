#getirSU Challenge Implementation //January, 2021
from gurobipy import GRB,Model,quicksum #*
import numpy as np
import json


def SU_Gurobi_Model(data):
    # Create the model
    suModel = Model('Getir')
    # Set parameters
    suModel.setParam('OutputFlag', True)
    
    n = len(data['vehicles']) #number of vehicles
    m = len(data['jobs'])     #number of jobs 
    
    S = range(0,n)          # source nodes of the vehicles 
    J = range(n,n+m)        # job or customer nodes
    V = range(0,n)          # vehicles
    S_J = range(0,n+m)      # sources and jobs
    J_E = range(n, n+m+m)   # jobs and ARTIFICIAL ending nodes
    
    # Add variables
    x = suModel.addVars(S_J,J_E, V, vtype = GRB.BINARY, name='x')              #assigment variable - between arcs(locations)
    u = suModel.addVars(J_E, V, lb = 2, ub = m, vtype = GRB.INTEGER, name='u') # sequence variable which is required for subtour elimination
    
    # minimize the total travel time
    suModel.setObjective(quicksum(data['matrix'][i][j]*x[i,j,k] for i in S_J for j in J_E for k in V if j<n+m), GRB.MINIMIZE)
    
    
    suModel.addConstrs((quicksum(x[i,j,k] for j in J) == 1 for i in S for k in V if i==k ), name='c1')
    suModel.addConstrs((quicksum(x[i,j,k] for j in J) == 1 for i in S for k in V if i==k ), name='c2')
    suModel.addConstrs((quicksum(x[i,i+m,k] for i in J) == 1 for k in V), name='c3')
    
    suModel.addConstrs((quicksum(x[i,j,k] for i in S_J for k in V if i!=j ) == 1 for j in J ), name='c4')
   
    suModel.addConstrs((quicksum(x[i,j,k] for i in S_J) == quicksum(x[j,h,k] for h in J_E) for k in V for j in J), name='c5')
    #subtour eliminations
    suModel.addConstrs( (u[i,k]-u[j,k]+m*x[i,j,k] <= (m-1) for i in J for j in J_E for k in V if i!=j), name='c6')
    
    # Optimize the model
    suModel.optimize()
    # write the results
    #OutputDictionary(suModel,data)
    Output(suModel)
    # print the LP file   (this file can be used in different optimization platforms such as CPLEX, Gams, etc.)
    suModel.write('suGetir.lp')
    # print the sol file
    suModel.write('suGetir.sol')
    
    routes = {}
    solution = suModel.getAttr('x',x)
    for k in V:
        route = []
        for i in S:
            for j in J:
                if solution[i, j, k] == 1:
                    route.append(i)
                    route.append(j)
        while route[-1] < n+m:
            for j in J_E:
                if route[-1] < n+m:
                    if solution[route[-1],j,k] == 1:
                        route.append(j)
        routes[str(k)] = route[0:len(route)-1]
    print(routes)

def parse_json(fileName):
   fObj = open(fileName,)
   ogdata = json.load(fObj)
   print(ogdata)
   return ogdata

def Output(m):  
    # Print the result
    status_code = {1:'LOADED', 2:'OPTIMAL', 3:'INFEASIBLE', 4:'INF_OR_UNBD', 5:'UNBOUNDED'}                                                                                      
    status = m.status
    
    print('\nThe optimization status is ' + status_code[status])
    if status == 2:    
        # Retrieve variables value
        print('Optimal primal solution:')
        for l in range(0,3):
            for v in m.getVars():
                if(v.x == 1 and int(v.varName[-2]) == l):
                    #print(str(v.varName) + " = " + str(v.x))
                    print("{}: {}".format(v.varName, v.X))
        print('Optimal objective value: ' + str(m.objVal) + "\n")
        
  
#MAIN FUNCTION#
H2O = SU_Gurobi_Model(parse_json('getir_algo_input.json'))


