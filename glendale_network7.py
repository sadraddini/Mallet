from macro_main import *

L={}
for i in range(1,113):
    L[i]=link(str(i))


N=network()
N.links=L.values()


f = open ( 'network_7by7_links.txt' , 'r')
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
f = open ( 'volume_tau30_7by7_random_2_1.txt' , 'r')
rawX = [ map(int,line.split('	')) for line in f ]
for t in range(1,len(rawX)+1):
    for i in range(1,113):
        xData[L[i],t]=rawX[t-1][i-1] 
f.close()


uData={}
# uData: signal
f = open ( 'signals_7by7_links_2.txt' , 'r')
rawU = [ map(float,line.split('	')) for line in f ]
for t in range(1,len(rawU)+1):
    print t
    for i in range(1,113):
        print i,
        uData[L[i],t]=rawU[t-1][i-1]
f.close()

# Debugging: can volume decrease with red light?
if True:
    NData=max(l[1] for l in uData.keys())
    print "Data Size",NData
    for t in range(1,NData):
        for l in N.links:
            if l.type=="road":
                if uData[l,t]==0 and xData[l,t]-xData[l,t+1]>=3:
                    print "Error found"
                    print "link=",l,"time=",t,xData[l,t],"to",xData[l,t+1]
                    raise "Error! correct this!"
                
if True:
    for l in N.links:
        print "\n","*"*80,"\n",l,
        for t in range(1,NData):
            print t,xData[l,t],"\t",    
 
if True:
    for l in N.links:
        print "\n","*"*80,"\n",l,
        for t in range(1,NData):
            print t,uData[l,t],"\t",
            
               
if False:
    for l in N.links:
        print l,l.type,"incoming:",l.incoming,"\toutgoing", l.outgoing   


N.tau=30.0



# Arrival rates 
f = open ( 'inflows_7by7_links.txt' , 'r')
row = [ map(int,line.split('	')) for line in f ]
print "\n\n",row
print len(row)
for r in range(0,len(row)):
    L[row[r][0]].lambda_arrival=row[r][1]/3600.0*N.tau
f.close()


        

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

# N.antagonistics=[(L[1],L[2]),(L[6],L[4]),(L[8],L[7]),(L[10],L[12]),(L[13],L[14]),(L[18],L[16]),(L[20],L[19])]
# N.antagonistics=.extend[(L[23),L[24]),(L[17],L[25]),(L[28],L[27]),(L[29],L[11]),(L[31],L[32]),(L[33],L[5]),(L[35],L[36]]
k=0
row1=[(L[1+k],L[2+k]),(L[6+k],L[4+k]),(L[8+k],L[7+k]),(L[10+k],L[12+k]),(L[13+k],L[14+k]),(L[18+k],L[16+k]),(L[20+k],L[19+k])]
row2=[(L[23+k],L[24+k]),(L[17+k],L[25+k]),(L[28+k],L[27+k]),(L[29+k],L[11+k]),(L[31+k],L[32+k]),(L[33+k],L[5+k]),(L[35+k],L[36+k])]
k=0
row3=[(L[38+k],L[39+k]),(L[40+k],L[34+k]),(L[42+k],L[43+k]),(L[44+k],L[30+k]),(L[46+k],L[47+k]),(L[48+k],L[26+k]),(L[50+k],L[51+k])]
k=0
row4=[(L[53+k],L[54+k]),(L[55+k],L[49+k]),(L[57+k],L[58+k]),(L[59+k],L[45]),(L[61+k],L[62+k]),(L[63+k],L[41+k]),(L[65+k],L[66+k])]
k=0
row5=[(L[68+k],L[69+k]),(L[70+k],L[64+k]),(L[72+k],L[73+k]),(L[74+k],L[60+k]),(L[76],L[77+k]),(L[78+k],L[56+k]),(L[80+k],L[81+k])]
k=0
row6=[(L[83+k],L[84]),(L[85+k],L[79+k]),(L[87+k],L[88+k]),(L[89+k],L[75+k]),(L[91+k],L[92+k]),(L[93+k],L[71]),(L[95+k],L[96+k])]
k=0
row7=[(L[98+k],L[99]),(L[100+k],L[94+k]),(L[102],L[103]),(L[104+k],L[90+k]),(L[106+k],L[107+k]),(L[108+k],L[86]),(L[110+k],L[111+k])]
N.antagonistics=row1+row2+row3+row4+row5+row6+row7
for v in N.antagonistics:
    print v
N.control_default()