from macro_main import *

L={}
for i in range(1,13):
    L[i]=link(str(i))

L[2].type="out"
L[3].type="out"
L[8].type="out"
L[10].type="out"



N=network()
N.links=L.values()
N.link2link(L[1],L[8])
N.link2link(L[1],L[6])

N.link2link(L[4],L[2])
N.link2link(L[4],L[9])

N.link2link(L[5],L[6])
N.link2link(L[5],L[8])

N.link2link(L[6],L[11])
N.link2link(L[6],L[3])

N.link2link(L[7],L[1])
N.link2link(L[7],L[10])

N.link2link(L[9],L[1])
N.link2link(L[9],L[10])

N.link2link(L[11],L[2])
N.link2link(L[11],L[9])

N.link2link(L[12],L[11])
N.link2link(L[12],L[3])

# xData 
xData={}
f = open ( 'volume_inflow900_tau5_noturn_6.txt' , 'r')
rawX = [ map(int,line.split('	')) for line in f ]
for t in range(1,len(rawX)+1):
    for i in range(1,13):
        xData[L[i],t]=rawX[t-1][i-1] 
f.close()

uData={}
# uData: signal
f1 = open ( 'signal_sg1_inflow900_tau5_noturn_6.txt' , 'r')
rawU1 = [ map(float,line.split('	')) for line in f1 ]
f2 = open ( 'signal_sg2_inflow900_tau5_noturn_6.txt' , 'r')
rawU2 = [ map(float,line.split('	')) for line in f2 ]
i=1
k=1
for t in range(1,len(rawU1[0])+k):
    uData[L[1],t]=rawU1[3-i][t-k]
    uData[L[2],t]=1
    uData[L[3],t]=1
    uData[L[4],t]=rawU1[1-i][t-k]
    uData[L[5],t]=rawU2[3-i][t-k]
    uData[L[6],t]=rawU2[2-i][t-k]
    uData[L[7],t]=rawU1[4-i][t-k]
    uData[L[8],t]=1
    uData[L[9],t]=rawU2[4-i][t-k]
    uData[L[10],t]=1
    uData[L[11],t]=rawU2[1-i][t-k]
    uData[L[12],t]=rawU1[2-i][t-k]
f.close()


# Debugging: can volume decrease with red light?
if True:
    NData=max(l[1] for l in xData.keys())
    print "Data Size",NData
    for t in range(1,NData):
        for l in N.links:
            if l.type=="road":
                if uData[l,t]==0 and xData[l,t]-xData[l,t+1]>=3:
                    print "Error found"
                    print l,t,xData[l,t],"to",xData[l,t+1]
                    raise "Error! correct this!"
                
    
    
if True:
    for l in N.links:
        print l,l.type,"incoming:",l.incoming,"\toutgoing", l.outgoing   







for i in [7,12,5,4]:
    L[i].lambda_arrival=900.0/3600*N.tau
    
# Initial guesses for turning ratios
for l in N.links:
    for k in l.outgoing:
        N.beta[l,k]=1.0/len(l.outgoing)
    for k in l.incoming:
        N.alpha[k,l]=1.0/len(l.incoming)
        

N.MILP_1(xData,uData)
N.MILP_2(xData,uData) 
N.MILP_1(xData,uData)
N.MILP_2(xData,uData) 
N.MILP_1(xData,uData)

for l in N.links:
    if l.type=="road":
        print l, "disturbance=",l.d, "\t free velocity=[",l.v_low,",",l.v_up,"]\t congested speed=[",l.w_low,",",l.w_up,"]\t congested speed*capacity=[",l.wc_low,",",l.wc_up,"]"   
    
for l in N.links:
    for k in l.outgoing:
        print "beta from ",l," to ",k,":\t",N.beta[l,k]
    for k in l.incoming:
        print "alpha from ",k," to ",l,":\t",N.alpha[k,l]
