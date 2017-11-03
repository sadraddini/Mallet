from gurobipy import *

class link:
    def __init__(self,name):
        self.x=0  # State of the link
        self.type="road"
        self.cap=0 # Capacity of the link
        self.v_low=0 # Lower Bound of Free flow speed
        self.v_up=0 # Upper Bound of Free flow speed
        self.w_low=0 # Lower bound of congested speed
        self.w_up=0 # Upper bound of congested speed
        self.wc_low=0 # Lower Bound of Congestion speed*capacity
        self.wc_up=0 # Upper bound of congestion speed*capacity
        self.outgoing=[] # List of the outgoing links
        self.incoming=[] # List of the incoming links 
        self.u=1 # The control actuation 
        self.d=0 # maximal disturbance
        self.lambda_arrival=0 # The arrival rate
        self.L=120.0 # Link length
        self.M=0
        
        self.name=name # in case if you want to name a link!
        self.lane=1 # number of lanes. for graphical purposes
        self.head=(0,0) #head intersection coordinates
        self.tail=(0,0) #tail intersection coordinates
        
    def __str__(self):
        return str(self.name)
    
    def __repr__(self):
        return str(self.name)
        
    def update_link_up(self):
        for L in self.down:
            if not(self in L.up):
                    (L.up).append(self)

    def update_link_adj(self):
        for L in self.up:
            for J in L.down:
                if not(J in self.adj) and J!=self:
                    (self.adj).append(J)    
    def get_up(self):
        return self.up

    def get_down(self):
        return self.down
                
    def get_adj(self):
        return self.adj
        
    def get_x(self):
        return self.x
        
class network:
    def __init__(self):
        self.links=[]
        self.alpha={}
        self.beta={}
        self.M={}
        self.c={}
        self.tau=10.0
    
    def link2link(self,l1,l2): # Flow from l1 to l2
        l1.outgoing.append(l2)
        l2.incoming.append(l1)
        
                
    
    def MILP_1(self,xData,uData):
        """
            Here I fix turning ratios and find parameters
            Data= Dictionary
        """
        print "\n","*"*80,"\n","MILP 1: Parameter Estimation\n","*"*80        
        model=Model("parameters")
        outflow={}
        d={}
        bigM=500
        Q_out={}
        Q_in={}
        N=max(l[1] for l in xData.keys())
        print "x Data size is",N
        N=max(l[1] for l in uData.keys())
        print "u Data size is",N
        for l in self.links:
            l.d=model.addVar(lb=0,ub=200,obj=0*l.type=="road")  
            for t in range(1,N):
                d[l,t]=model.addVar(lb=0,ub=200,obj=1)
                for k in l.outgoing:
                    outflow[l,k,t]=model.addVar(lb=0,ub=200)
                    self.c[l,k]=model.addVar(lb=20,ub=200)
                    self.beta[l,k]=model.addVar(lb=0.2,ub=0.8)
                    self.alpha[l,k]=model.addVar(lb=0,ub=1)
                    self.M[l,k]=model.addVar(lb=0,ub=200)
                    d["outflow-1",l,k,t]=model.addVar(vtype=GRB.BINARY) 
                    d["outflow-2",l,k,t]=model.addVar(vtype=GRB.BINARY)        
        model.update()
        for t in range(1,N):
            for l in self.links:
                if True:
                    Q_out[l,t]=LinExpr()
                    Q_in[l,t]=LinExpr()
                    Q_out[l,t].addConstant(0)
                    Q_in[l,t].addConstant(0)
                    for k in l.outgoing:
                        model.addConstr(outflow[l,k,t]<=self.beta[l,k]*uData[l,t]*xData[l,t])
                        model.addConstr(outflow[l,k,t]<=self.M[l,k])
                        model.addConstr(outflow[l,k,t]<=self.c[l,k]-self.alpha[l,k]*xData[k,t])
                        model.addConstr(outflow[l,k,t]>=self.beta[l,k]*uData[l,t]*xData[l,t]+bigM*d["outflow-1",l,k,t]-bigM)
                        model.addConstr(outflow[l,k,t]>=self.M[l,k]+bigM*d["outflow-2",l,k,t]-bigM)
                        model.addConstr(outflow[l,k,t]>=self.c[l,k]-self.alpha[l,k]*xData[k,t]-bigM*d["outflow-1",l,k,t]-bigM*d["outflow-2",l,k,t])
                        Q_out[l,t].add(outflow[l,k,t])
                    for k in l.incoming:
                        Q_in[l,t].add(outflow[k,l,t])
                if l.type=="road":
                    model.addConstr(xData[l,t+1]<=xData[l,t]- Q_out[l,t] + Q_in[l,t] + d[l,t] + l.lambda_arrival)  
                else:
                    model.addConstr(xData[l,t+1]<=xData[l,t]- uData[l,t]*xData[l,t] + Q_in[l,t] + d[l,t] + l.lambda_arrival)
        for l in self.links:
            sum=LinExpr()
            for k in l.outgoing:
                sum.add(self.beta[l,k])
            model.addConstr(sum>=0)
            
#         J=QuadExpr()
#         for l in self.links:
#             for t in range(1,N):
#                 if l.type=="road":
#                     J.add(d[l,t]*d[l,t])
#         model.setObjective(J)
        model.optimize()
        for l in self.links:
            l.d=l.d.X
            for k in l.outgoing:
                self.beta[l,k]=self.beta[l,k].X
                self.c[l,k]=self.c[l,k].X
                self.alpha[l,k]=self.alpha[l,k].X
                self.M[l,k]=self.M[l,k].X
        for l in self.links:
            for t in range(1,N):
                l.d=max(d[l,t].X,l.d)
                
            

        
        if True:
            for t in range(1,N):
                print "*"*80,"time=",t
                for l in self.links:
                    print "\n",l,"x is",xData[l,t],"u is",uData[l,t],"x+ is",xData[l,t+1]
                    for k in l.outgoing:
                        print k,"beta:",self.beta[l,k],"outflow",outflow[l,k,t].X             
                        
                    
                
        

        