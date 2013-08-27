from motionplanning.ASVPlanner import ASVPlanner

testcases = ['3ASV-t1.txt', '3ASV-variable-x4.txt'] 
tuningPara = [ [(100,0.2), (150,0.15)], [(100,0.15), (200,0.1)] ]

for i, case in enumerate(testcases):
    paras = tuningPara[i]
    for para in paras: # may try different samples and radius
        sampleNum, radius = para
        ASVPlanner(case, sampleNum, radius)
