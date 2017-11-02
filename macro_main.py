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
        self.tau=10.0
    
    def link2link(self,l1,l2): # Flow from l1 to l2
        l1.outgoing.append(l2)
        l2.incoming.append(l1)
        
                
    
    def MILP_1(self,xData,uData):
        """
            Here I fix turning ratios and find parameters
            Data= Dictionary
                rho[l][t]: density
                u[l][t]: controls
        """
        print "\n","*"*80,"\n","MILP 1: Velocities\n","*"*80        
        model=Model("parameters")
        outflow={}
        inflow={}
        d={}
        bigM=1000
        N=max(l[1] for l in xData.keys())
        print "x Data size is",N
        N=max(l[1] for l in uData.keys())
        print "u Data size is",N
        for l in self.links:
            l.v_low=model.addVar(lb=0,ub=200)
            l.v_up=model.addVar(lb=0,ub=200)
            l.w_low=model.addVar(lb=0,ub=200)
            l.w_up=model.addVar(lb=0,ub=200)            
            l.wc_up=model.addVar(lb=0,ub=200)
            l.wc_low=model.addVar(lb=0,ub=200)
            l.d=model.addVar(lb=0,ub=200,obj=1*l.type=="road")  
            for t in range(1,N):
                for k in l.incoming:
                    inflow[k,l,t]=model.addVar(lb=0,ub=200)
                    d["inflow",k,l,t]=model.addVar(vtype=GRB.BINARY)
                for k in l.outgoing:
                    outflow[l,k,t]=model.addVar(lb=0,ub=200)
                    d["outflow",l,k,t]=model.addVar(vtype=GRB.BINARY)        
        model.update()
        for t in range(1,N):
            for l in self.links:
                if l.type=="road":
                    Q_out=LinExpr()
                    Q_in=LinExpr()
                    Q_out.addConstant(0)
                    Q_in.addConstant(0)
                    for k in l.outgoing:
                        model.addConstr(outflow[l,k,t]<=uData[l,t]*xData[l,t]*l.v_low)
                        model.addConstr(outflow[l,k,t]<=k.wc_low - xData[k,t]*k.w_low)
                        model.addConstr(outflow[l,k,t]>=uData[l,t]*xData[l,t]*l.v_low+bigM*d["outflow",l,k,t]-bigM)
                        model.addConstr(outflow[l,k,t]>=k.wc_low - xData[k,t]*k.w_low-bigM*d["outflow",l,k,t])
                        Q_out.add(self.tau*self.beta[l,k]*outflow[l,k,t])
                    for k in l.incoming:
                        model.addConstr(inflow[k,l,t]<=uData[k,t]*xData[k,t]*k.v_up)
                        model.addConstr(inflow[k,l,t]<=l.wc_up-xData[l,t]*l.w_up/l.L)
                        model.addConstr(inflow[k,l,t]>=uData[k,t]*xData[k,t]*k.v_up+bigM*d["inflow",k,l,t]-bigM)
                        model.addConstr(inflow[k,l,t]>=l.wc_up-xData[l,t]*l.w_up-bigM*d["inflow",k,l,t])
                        Q_in.add(self.tau*self.alpha[k,l]*inflow[k,l,t])
                    model.addConstr(xData[l,t+1]<=xData[l,t]-Q_out+Q_in+l.d+l.lambda_arrival)
#         J=QuadExpr()
#         for l in self.links:
#             if l.type=="road":
#                 model.addConstr(l.v_up==l.v_low)
#                 model.addConstr(l.w_up==l.w_low)
#                 model.addConstr(l.wc_up==l.wc_low)
#                 J.add(l.d)
#         model.setObjective(J)   
        model.optimize()
        for l in self.links:
            l.v_low=l.v_low.X
            l.v_up=l.v_up.X
            l.w_low=l.w_low.X
            l.w_up=l.w_up.X         
            l.wc_up=l.wc_up.X
            l.wc_low=l.wc_low.X
            l.d=l.d.X
        
        if True:
            for t in range(1,N):
                print "*"*80,"time=",t
                for l in self.links:
                    print "\n",l,"x is",xData[l,t],"u is",uData[l,t],"x+ is",xData[l,t]
                    for k in l.outgoing:
                        print k,"beta:",self.beta[l,k],"outflow",outflow[l,k,t].X 
                    for k in l.incoming:
                        print k,"alpha:",self.alpha[k,l],"inflow",inflow[k,l,t].X             
            
                        
                    
                
        
    def MILP_2(self,xData,uData):
        """
            Here I fix parameters and find turning ratios
        """
        print "\n","*"*80,"\n","MILP 2: Turning rations\n","*"*80        
        model=Model("parameters")
        outflow={}
        inflow={}
        d={}
        bigM=1000
        beta={}
        alpha={}
        N=max(l[1] for l in xData.keys())
        print "x Data size is",N
        N=max(l[1] for l in uData.keys())
        for l in self.links:
            for k in l.incoming:
                alpha[k,l]=model.addVar(lb=0,ub=1)
            for k in l.outgoing:
                beta[l,k]=model.addVar(lb=0,ub=1)           
            l.d=model.addVar(lb=0,ub=200,obj=1*l.type=="road")       
        model.update()
        for t in range(1,N):
            for l in self.links:
                if l.type=="road":
                    Q_out=LinExpr()
                    Q_in=LinExpr()
                    for k in l.outgoing:
                        outflow[l,k,t]=min(uData[l,t]*xData[l,t]*l.v_low,k.wc_low-xData[k,t]*k.w_low)
                        Q_out.add(self.tau*beta[l,k]*outflow[l,k,t])
                    for k in l.incoming:
                        inflow[k,l,t]=min(uData[k,t]*xData[k,t]*k.v_up,l.wc_up-xData[l,t]*l.w_up)
                        Q_in.add(self.tau*alpha[k,l]*inflow[k,l,t])
                    model.addConstr(xData[l,t+1]<=xData[l,t]-Q_out+Q_in+l.d+l.lambda_arrival)
        for l in self.links:
            sum=LinExpr()
            for k in l.outgoing:
                sum.add(beta[l,k])
            model.addConstr(sum<=1)
#         for l in self.links:
#             sum=LinExpr()
#             for k in l.incoming:
#                 sum.add(alpha[k,l])
#             model.addConstr(sum==1)        
        model.optimize()
        for l in self.links:
            l.d=l.d.X
            for k in l.outgoing:
                self.beta[l,k]=beta[l,k].X
            for k in l.incoming:
                self.alpha[k,l]=alpha[k,l].X
        