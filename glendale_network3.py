from macro_main import *

L={}
for i in range(1,25):
    L[i]=link(str(i))


N=network()
N.links=L.values()


f = open ( 'network_3by3_links.txt' , 'r')
raw = [ map(int,line.split('	')) for line in f ]
for i in range(1,len(raw)+1):
    for j in range(0,2):
        k=raw[i-1][j]
        if k>0:
            N.link2link(L[i],L[k])
        else:
            L[i].type="out"
f.close()

for l in N.links:
    print l, l.type,"\t", l.outgoing




# xData 
xData={}
f = open ( 'volume_tau30_3by3_random_1.txt' , 'r')
rawX = [ map(int,line.split('	')) for line in f ]
for t in range(1,len(rawX)+1):
    for i in range(1,25):
        xData[L[i],t]=rawX[t-1][i-1] 
f.close()




uData={}
# uData: signal
f = open ( 'signals_3by3_links_1.txt' , 'r')
rawU = [ map(float,line.split('	')) for line in f ]
for t in range(1,len(rawU)+1):
    for i in range(1,25):
        uData[L[i],t]=rawU[t-1][i-1]
f.close()

# Debugging: can volume decrease with red light?
if True:
    NData=max(l[1] for l in uData.keys())
    print "Data Size",NData
    for t in range(1,NData):
        for l in N.links:
            if l.type=="road":
                if uData[l,t]==0 and xData[l,t]-xData[l,t+1]>=2:
                    print "Error found"
                    print l,t,xData[l,t],"to",xData[l,t+1]
                    raise "Error! correct this!"
                
if False:
    for l in N.links:
        print "\n","*"*80,"\n",l,
        for t in range(1,NData):
            print t,xData[l,t],"\t",    
 
if False:
    for l in N.links:
        print "\n","*"*80,"\n",l,
        for t in range(1,NData):
            print t,uData[l,t],"\t",
            
               
if True:
    for l in N.links:
        print l,l.type,"incoming:",l.incoming,"\toutgoing", l.outgoing   


N.tau=30.0

# for i in [7,12,5,4]:
#     L[i].lambda_arrival=1000.0/3600.0*N.tau
        

N.MILP_1(xData,uData)


for l in N.links:
    if l.type=="road":
        print l, "average disturbance=",l.d
        print l, "maximal disturbance=",l.dstar
    
for l in N.links:
    for k in l.outgoing:
        print "alpha from ",l," to ",k,":\t",N.alpha[l,k]
        print "beta from ",l," to ",k,":\t",N.beta[l,k]
        print "M from ",l," to ",k,":\t",N.M[l,k]
        print "c from ",l," to ",k,":\t",N.c[l,k]

if False:
    for t in range(1,180):
        print "*"*80,"\n",t,"\n","*"*80
        for l in N.links:
            print l, xData[l,t], "\t control:",uData[l,t]